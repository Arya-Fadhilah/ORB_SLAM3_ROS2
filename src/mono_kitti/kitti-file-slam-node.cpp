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
