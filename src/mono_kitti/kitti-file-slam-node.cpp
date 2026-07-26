// =============================================================================
// STATUS: Debug/testing tool SEMENTARA. Lihat catatan lengkap di
// kitti-file-slam-node.hpp sebelum menganggap ini pengganti node produksi.
// =============================================================================

#include "kitti-file-slam-node.hpp"

#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

KittiFileSlamNode::KittiFileSlamNode(ORB_SLAM3::System* pSLAM, const std::string& strSequencePath, const std::string& strTimesFile)
:   Node("ORB_SLAM3_KITTI_FILE_DEBUG")
{
    m_SLAM = pSLAM;

    m_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local());
    m_map_timer = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&KittiFileSlamNode::PublishOccupancyGrid, this));
    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", rclcpp::SensorDataQoS());
    m_pointcloud_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),   // 5Hz — jauh lebih sering dari grid, karena datanya kecil
        std::bind(&KittiFileSlamNode::PublishPointCloud, this));

    LoadImages(strSequencePath, strTimesFile, m_vstrImageFilenames, m_vTimestamps);

    std::cout << "KittiFileSlamNode: loaded " << m_vstrImageFilenames.size() << " images" << std::endl;
}

KittiFileSlamNode::~KittiFileSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SavePointCloud("PointCloud.ply");
}

void KittiFileSlamNode::LoadImages(const std::string& strSequencePath, const std::string& strTimesFile,
                                     std::vector<std::string>& vstrImageFilenames, std::vector<double>& vTimestamps)
{
    std::ifstream fTimes(strTimesFile);
    if (!fTimes.is_open())
    {
        std::cerr << "ERROR: cannot open times file: " << strTimesFile << std::endl;
        return;
    }

    while (!fTimes.eof())
    {
        std::string s;
        std::getline(fTimes, s);
        if (!s.empty())
        {
            std::stringstream ss(s);
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    fTimes.close();

    // Asumsi format KITTI standar: 000000.png, 000001.png, dst (6 digit, zero-padded)
    // SESUAIKAN kalau struktur folder dataset kamu beda.
    vstrImageFilenames.resize(vTimestamps.size());
    for (size_t i = 0; i < vTimestamps.size(); i++)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%06zu.png", i);
        vstrImageFilenames[i] = strSequencePath + "/" + std::string(buf);
    }
}

void KittiFileSlamNode::RunSequence()
{
    const size_t nImages = m_vstrImageFilenames.size();

    for (size_t ni = 0; ni < nImages && rclcpp::ok(); ni++)
    {
        cv::Mat im = cv::imread(m_vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
        if (im.empty())
        {
            std::cerr << "Failed to load: " << m_vstrImageFilenames[ni] << std::endl;
            break;
        }

        double tframe = m_vTimestamps[ni];

        std::cout << "processing frame " << ni << ", stamp=" << tframe << std::endl;
        m_SLAM->TrackMonocular(im, tframe);

        // Proses callback ROS2 (timer publish /map tiap 2 detik) di antara frame
        rclcpp::spin_some(this->get_node_base_interface());
    }

    std::cout << "RunSequence finished." << std::endl;
}

void KittiFileSlamNode::PublishOccupancyGrid()
{
    cv::Mat grid = m_SLAM->GetOccupancyGrid();
    if (grid.empty()) return;

    float resolution, originX, originY;
    int width, height;
    if (!m_SLAM->GetOccupancyGridMetadata(resolution, originX, originY, width, height))
        return;

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    msg.info.resolution = resolution;
    msg.info.width = width;
    msg.info.height = height;
    msg.info.origin.position.x = originX;
    msg.info.origin.position.y = originY;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(width * height);
    for (int y = 0; y < height; ++y) {
        int cvRow = height - 1 - y;
        for (int x = 0; x < width; ++x) {
            msg.data[y * width + x] = static_cast<int8_t>(grid.at<schar>(cvRow, x));
        }
    }

    m_map_publisher->publish(msg);
}

std::vector<Eigen::Vector3f> KittiFileSlamNode::FilterOutliersMAD(
    const std::vector<Eigen::Vector3f>& pts, float threshold)
{
    if (pts.size() < 10) return pts;

    auto median = [](std::vector<float> v) {
        size_t n = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + n, v.end());
        return v[n];
    };

    std::vector<float> xs, ys, zs;
    for (auto& p : pts) { xs.push_back(p.x()); ys.push_back(p.y()); zs.push_back(p.z()); }
    Eigen::Vector3f centroid(median(xs), median(ys), median(zs));

    std::vector<float> dists;
    for (auto& p : pts) dists.push_back((p - centroid).norm());
    float medDist = median(dists);

    std::vector<float> devs;
    for (float d : dists) devs.push_back(std::fabs(d - medDist));
    float mad = median(devs) + 1e-6f;

    std::vector<Eigen::Vector3f> clean;
    for (size_t i = 0; i < pts.size(); i++) {
        if (std::fabs(dists[i] - medDist) / mad > threshold) continue;
        clean.push_back(pts[i]);
    }
    return clean;
}

void KittiFileSlamNode::PublishPointCloud()
{
    if (m_pointcloud_publisher->get_subscription_count() == 0) return;

    std::vector<ORB_SLAM3::MapPoint*> vpMPs = m_SLAM->GetTrackedMapPoints();
    if (vpMPs.empty()) return;

    std::vector<Eigen::Vector3f> vPos;
    vPos.reserve(vpMPs.size());
    for (auto* pMP : vpMPs) {
        if (!pMP || pMP->isBad()) continue;
        Eigen::Vector3f p = pMP->GetWorldPos();
        if (!p.allFinite()) continue;
        vPos.push_back(p);
    }
    if (vPos.empty()) return;

    std::vector<Eigen::Vector3f> vClean = FilterOutliersMAD(vPos);
    if (vClean.empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";   // konsisten dengan PublishOccupancyGrid() yang sudah jalan

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(vClean.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    for (size_t i = 0; i < vClean.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = vClean[i].x();
        *iter_y = vClean[i].y();
        *iter_z = vClean[i].z();
    }

    m_pointcloud_publisher->publish(msg);
}