// =============================================================================
// STATUS: BELUM DITEST. Lihat catatan lengkap di mono-inertial-file-slam-node.hpp
//
// Loading pattern (LoadImages/LoadIMU) diadaptasi dari
// Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc yang sudah ada di
// project knowledge kamu (LoadImagesTUMVI/LoadIMU) — format timestamp gambar
// dalam nanodetik (dibagi 1e9), format IMU CSV: t,gx,gy,gz,ax,ay,az.
// =============================================================================

#include "mono-inertial-file-slam-node.hpp"

#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

MonoInertialFileSlamNode::MonoInertialFileSlamNode(ORB_SLAM3::System* pSLAM,
                                                    const std::string& strImagePath,
                                                    const std::string& strImageTimesFile,
                                                    const std::string& strImuPath)
:   Node("ORB_SLAM3_MONO_INERTIAL_FILE_DEBUG")
{
    m_SLAM = pSLAM;

    m_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local());
    m_map_timer = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&MonoInertialFileSlamNode::PublishOccupancyGrid, this));

    LoadImages(strImagePath, strImageTimesFile, m_vstrImages, m_vTimestampsImg);
    LoadIMU(strImuPath, m_vTimestampsImu, m_vAcc, m_vGyro);

    std::cout << "MonoInertialFileSlamNode: loaded " << m_vstrImages.size()
              << " images, " << m_vTimestampsImu.size() << " IMU samples" << std::endl;
}

MonoInertialFileSlamNode::~MonoInertialFileSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SavePointCloud("PointCloud.ply");
}

void MonoInertialFileSlamNode::LoadImages(const std::string& strImagePath, const std::string& strImageTimesFile,
                                            std::vector<std::string>& vstrImages, std::vector<double>& vTimestampsImg)
{
    std::ifstream fTimes(strImageTimesFile);
    if (!fTimes.is_open())
    {
        std::cerr << "ERROR: cannot open image times file: " << strImageTimesFile << std::endl;
        return;
    }

    vTimestampsImg.reserve(5000);
    vstrImages.reserve(5000);

    while (!fTimes.eof())
    {
        std::string s;
        std::getline(fTimes, s);
        if (!s.empty())
        {
            if (s[0] == '#') continue;  // skip komentar, pola TUM-VI

            size_t pos = s.find(' ');
            std::string item = s.substr(0, pos);

            vstrImages.push_back(strImagePath + "/" + item + ".png");

            double t = std::stod(item);
            vTimestampsImg.push_back(t / 1e9);  // nanodetik -> detik, pola TUM-VI
        }
    }
    fTimes.close();
}

void MonoInertialFileSlamNode::LoadIMU(const std::string& strImuPath,
                                        std::vector<double>& vTimestampsImu,
                                        std::vector<cv::Point3f>& vAcc,
                                        std::vector<cv::Point3f>& vGyro)
{
    std::ifstream fImu(strImuPath);
    if (!fImu.is_open())
    {
        std::cerr << "ERROR: cannot open IMU file: " << strImuPath << std::endl;
        return;
    }

    vTimestampsImu.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while (!fImu.eof())
    {
        std::string s;
        std::getline(fImu, s);
        if (s.empty()) continue;
        if (s[0] == '#') continue;

        std::string item;
        size_t pos = 0;
        double data[7];
        int count = 0;
        while ((pos = s.find(',')) != std::string::npos && count < 6)
        {
            item = s.substr(0, pos);
            data[count++] = std::stod(item);
            s.erase(0, pos + 1);
        }
        item = s.substr(0, pos);
        data[6] = std::stod(item);

        vTimestampsImu.push_back(data[0] / 1e9);
        vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
        vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
    }
    fImu.close();
}

void MonoInertialFileSlamNode::RunSequence()
{
    const size_t nImages = m_vstrImages.size();

    // Cari indeks IMU pertama yang timestamp-nya >= gambar pertama
    // (pola sama seperti Examples/Monocular-Inertial resmi ORB-SLAM3)
    size_t first_imu = 0;
    while (first_imu < m_vTimestampsImu.size() && m_vTimestampsImu[first_imu] <= m_vTimestampsImg[0])
        first_imu++;
    if (first_imu > 0) first_imu--;

    size_t imuIdx = first_imu;
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for (size_t ni = 0; ni < nImages && rclcpp::ok(); ni++)
    {
        cv::Mat im = cv::imread(m_vstrImages[ni], cv::IMREAD_UNCHANGED);
        if (im.empty())
        {
            std::cerr << "Failed to load: " << m_vstrImages[ni] << std::endl;
            break;
        }

        double tframe = m_vTimestampsImg[ni];

        // Kumpulkan semua sampel IMU sampai timestamp frame ini (pola sama
        // seperti Examples/Monocular-Inertial/mono_inertial_euroc.cc)
        vImuMeas.clear();
        while (imuIdx < m_vTimestampsImu.size() && m_vTimestampsImu[imuIdx] <= tframe)
        {
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                m_vAcc[imuIdx], m_vGyro[imuIdx], m_vTimestampsImu[imuIdx]));
            imuIdx++;
        }

        std::cout << "processing frame " << ni << ", stamp=" << tframe
                  << ", imu_samples=" << vImuMeas.size() << std::endl;

        m_SLAM->TrackMonocular(im, tframe, vImuMeas);

        rclcpp::spin_some(this->get_node_base_interface());
    }

    std::cout << "RunSequence finished." << std::endl;
}

void MonoInertialFileSlamNode::PublishOccupancyGrid()
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
