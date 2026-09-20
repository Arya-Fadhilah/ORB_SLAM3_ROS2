// =============================================================================
// Lihat catatan lengkap di mono-inertial-file-slam-node.hpp
//
// Loading pattern (LoadImages/LoadIMU) mengikuti mono_inertial_tum_vi.cc.
// Publisher point cloud / TF / konversi sumbu adalah salinan dari
// KittiFileSlamNode (lihat catatan duplikasi di header).
// =============================================================================

#include "mono-inertial-file-slam-node.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// Rotasi dari konvensi optical frame ORB-SLAM3 (x kanan, y bawah, z depan)
// ke konvensi ROS REP-103 (x depan, y kiri, z atas).
static const Eigen::Matrix3f R_OPT_TO_ROS = Eigen::Matrix3f::Identity();

MonoInertialFileSlamNode::MonoInertialFileSlamNode(ORB_SLAM3::System* pSLAM,
                                                    const std::string& strImagePath,
                                                    const std::string& strImageTimesFile,
                                                    const std::string& strImuPath,
                                                    const std::string& format)
:   Node("ORB_SLAM3_MONO_INERTIAL_FILE_DEBUG"),
    m_prev_stamp(0, 0, RCL_ROS_TIME)
{
    m_SLAM = pSLAM;
    m_format = format;

    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    PublishStaticMapToOdom();

    //m_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    //    "map", rclcpp::QoS(1).transient_local());
    //m_map_timer = this->create_wall_timer(
    //    std::chrono::seconds(2),
    //    std::bind(&MonoInertialFileSlamNode::PublishOccupancyGrid, this));

    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);
    m_pointcloud_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MonoInertialFileSlamNode::PublishPointCloud, this));

    m_pointcloud_full_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud_map", 10);
    m_pointcloud_full_timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MonoInertialFileSlamNode::PublishFullMapPointCloud, this));

    m_pointcloud_cam_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud_cam", 10);
    
    m_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    m_clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    
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

// =============================================================================
// Loading
// =============================================================================

void MonoInertialFileSlamNode::LoadImages(const std::string& strImagePath, const std::string& strImageTimesFile,
                                            std::vector<std::string>& vstrImages, std::vector<double>& vTimestampsImg)
{
    std::cout << "LoadImages: m_format=" << m_format << std::endl;
    
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

            if (m_format == "rover") {
                std::string fname = s.substr(pos + 1);    // kolom 2 = "rgb/xxx.png"
                size_t slash = fname.find_last_of('/');
                if (slash != std::string::npos) fname = fname.substr(slash + 1);
                // buang \r kalau file bergaya Windows
                while (!fname.empty() && (fname.back() == '\r' || fname.back() == '\n' || fname.back() == ' '))
                    fname.pop_back();
                vstrImages.push_back(strImagePath + "/" + fname);
                vTimestampsImg.push_back(std::stod(item));
            } else {
                vstrImages.push_back(strImagePath + "/" + item + ".png");
                vTimestampsImg.push_back(std::stod(item) / 1e9);
            }
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

        if (m_format == "rover") {
            vTimestampsImu.push_back(data[0]);
            vAcc.push_back(cv::Point3f(data[1], data[2], data[3]));
            vGyro.push_back(cv::Point3f(data[4], data[5], data[6]));
        } else {
            vTimestampsImu.push_back(data[0] / 1e9);
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
        }
    }
    fImu.close();
}

// =============================================================================
// Main loop
// =============================================================================

void MonoInertialFileSlamNode::RunSequence()
{
    const size_t nImages = m_vstrImages.size();
    if (nImages == 0) { std::cerr << "No images loaded, aborting." << std::endl; return; }

    size_t first_imu = 0;
    while (first_imu < m_vTimestampsImu.size() && m_vTimestampsImu[first_imu] <= m_vTimestampsImg[0])
        first_imu++;
    if (first_imu > 0) first_imu--;

    size_t imuIdx = first_imu;
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for (size_t ni = 0; ni < nImages && rclcpp::ok(); ni++)
    {
        if (m_SLAM->isShutDown()) break;
        
        // IMREAD_GRAYSCALE (bukan UNCHANGED) mengikuti mono_inertial_tum_vi.cc:
        // memaksa CV_8UC1 apapun bit-depth PNG-nya, karena ekstraktor ORB
        // mengharapkan 8-bit.
        cv::Mat im = cv::imread(m_vstrImages[ni], cv::IMREAD_GRAYSCALE);
        if (im.empty())
        {
            std::cerr << "Failed to load: " << m_vstrImages[ni] << std::endl;
            break;
        }

        if (m_format != "rover") m_clahe->apply(im, im);

        double tframe = m_vTimestampsImg[ni];

        vImuMeas.clear();
        while (imuIdx < m_vTimestampsImu.size() && m_vTimestampsImu[imuIdx] <= tframe)
        {
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                m_vAcc[imuIdx], m_vGyro[imuIdx], m_vTimestampsImu[imuIdx]));
            imuIdx++;
        }

        std::cout << "processing frame " << ni << ", stamp=" << tframe
                  << ", imu_samples=" << vImuMeas.size() << std::endl;

        if (ni < 5)
            std::cout << "  im: " << im.cols << "x" << im.rows
                      << " ch=" << im.channels() << " type=" << im.type() << std::endl;        
        
        Sophus::SE3f Tcw = m_SLAM->TrackMonocular(im, tframe, vImuMeas);

        if (ni % 30 == 0)
            std::cout << "  trackingState=" << m_SLAM->GetTrackingState() << std::endl;        

        if (m_SLAM->GetTrackingState() == 2)  // 2 = Tracking::OK
        {
            Sophus::SE3f Twc = Tcw.inverse();
            rclcpp::Time stamp = this->now();
            BroadcastOdomToBaseLink(Twc, stamp);
            PublishOdometry(Twc, stamp);
            PublishPointCloudCam(Tcw);
        }

        // Throttle ~20 Hz, mendekati laju asli TUM-VI. Tanpa ini kamera melesat
        // lebih cepat daripada costmap Nav2 sempat menggeser jendelanya.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        rclcpp::spin_some(this->get_node_base_interface());
    }

    std::cout << "RunSequence finished." << std::endl;
}

// =============================================================================
// Konversi sumbu
// =============================================================================

Eigen::Vector3f MonoInertialFileSlamNode::ConvertPointToRos(const Eigen::Vector3f &p_cam)
{
    return R_OPT_TO_ROS * p_cam;
}

void MonoInertialFileSlamNode::ConvertPoseToRos(const Sophus::SE3f &Twc_cam,
                                                 Eigen::Vector3f &trans_ros, Eigen::Quaternionf &q_ros)
{
    trans_ros = R_OPT_TO_ROS * Twc_cam.translation();
    Eigen::Matrix3f R_ros = R_OPT_TO_ROS * Twc_cam.rotationMatrix() * R_OPT_TO_ROS.transpose();
    q_ros = Eigen::Quaternionf(R_ros);
}

// =============================================================================
// TF
// =============================================================================

void MonoInertialFileSlamNode::PublishStaticMapToOdom()
{
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = this->now();
    t1.header.frame_id = "map";
    t1.child_frame_id = "odom";
    t1.transform.rotation.w = 1.0;
    transforms.push_back(t1);

    geometry_msgs::msg::TransformStamped t2;
    t2.header.stamp = this->now();
    t2.header.frame_id = "base_link_optical";
    t2.child_frame_id = "base_link";
    t2.transform.rotation.x = 0.5;
    t2.transform.rotation.y = -0.5;
    t2.transform.rotation.z = 0.5;
    t2.transform.rotation.w = 0.5;
    transforms.push_back(t2);

    m_static_tf_broadcaster->sendTransform(transforms);
}

void MonoInertialFileSlamNode::BroadcastOdomToBaseLink(const Sophus::SE3f &Twc, const rclcpp::Time &stamp)
{
    Eigen::Vector3f trans_ros;
    Eigen::Quaternionf q_ros;
    ConvertPoseToRos(Twc, trans_ros, q_ros);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link_optical";
    t.transform.translation.x = trans_ros.x();
    t.transform.translation.y = trans_ros.y();
    t.transform.translation.z = trans_ros.z();
    t.transform.rotation.x = q_ros.x();
    t.transform.rotation.y = q_ros.y();
    t.transform.rotation.z = q_ros.z();
    t.transform.rotation.w = q_ros.w();

    m_tf_broadcaster->sendTransform(t);
}

// =============================================================================
// /odom — finite difference dari pose SLAM
// =============================================================================

void MonoInertialFileSlamNode::PublishOdometry(const Sophus::SE3f &Twc, const rclcpp::Time &stamp)
{
    Eigen::Vector3f trans_ros;
    Eigen::Quaternionf q_opt;
    ConvertPoseToRos(Twc, trans_ros, q_opt);

    // Sama dengan static transform base_link_optical -> base_link.
    // Eigen::Quaternionf(w, x, y, z)
    static const Eigen::Quaternionf q_opt_to_base(0.5f, 0.5f, -0.5f, 0.5f);
    Eigen::Quaternionf q_ros = q_opt * q_opt_to_base;
    q_ros.normalize();

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = trans_ros.x();
    msg.pose.pose.position.y = trans_ros.y();
    msg.pose.pose.position.z = trans_ros.z();
    msg.pose.pose.orientation.x = q_ros.x();
    msg.pose.pose.orientation.y = q_ros.y();
    msg.pose.pose.orientation.z = q_ros.z();
    msg.pose.pose.orientation.w = q_ros.w();

    if (m_has_prev_pose)
    {
        double dt = (stamp - m_prev_stamp).seconds();
        if (dt > 1e-4)
        {
            // Kecepatan linear di frame base_link (bukan odom), sesuai konvensi
            // nav_msgs/Odometry: twist dinyatakan di child_frame_id.
            Eigen::Vector3f dp_odom = trans_ros - m_prev_trans;
            Eigen::Vector3f dp_body = q_ros.conjugate() * dp_odom;

            msg.twist.twist.linear.x = dp_body.x() / dt;
            msg.twist.twist.linear.y = dp_body.y() / dt;
            msg.twist.twist.linear.z = dp_body.z() / dt;

            Eigen::Quaternionf dq = m_prev_q.conjugate() * q_ros;
            dq.normalize();
            Eigen::AngleAxisf aa(dq);
            Eigen::Vector3f omega = aa.axis() * (aa.angle() / static_cast<float>(dt));

            msg.twist.twist.angular.x = omega.x();
            msg.twist.twist.angular.y = omega.y();
            msg.twist.twist.angular.z = omega.z();
        }
    }

    // Covariance placeholder. ASUMSI, bukan hasil karakterisasi: nilai besar
    // dipakai supaya konsumen hilir tidak menganggap odom ini presisi.
    for (int i = 0; i < 36; i++) { msg.pose.covariance[i] = 0.0; msg.twist.covariance[i] = 0.0; }
    for (int i = 0; i < 6; i++)  { msg.pose.covariance[i * 7] = 0.1; msg.twist.covariance[i * 7] = 0.1; }

    m_odom_publisher->publish(msg);

    m_prev_trans = trans_ros;
    m_prev_q = q_ros;
    m_prev_stamp = stamp;
    m_has_prev_pose = true;
}

// =============================================================================
// Point cloud
// =============================================================================

std::vector<Eigen::Vector3f> MonoInertialFileSlamNode::FilterOutliersMAD(
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

void MonoInertialFileSlamNode::PublishPointCloud()
{
    if (m_pointcloud_publisher->get_subscription_count() == 0) return;

    std::vector<ORB_SLAM3::MapPoint*> vpMPs = m_SLAM->GetTrackedMapPoints();
    if (vpMPs.empty()) return;

    std::vector<Eigen::Vector3f> vPos;
    vPos.reserve(vpMPs.size());
    for (auto* pMP : vpMPs) {
        if (!pMP || pMP->isBad()) continue;
        Eigen::Vector3f p = ConvertPointToRos(pMP->GetWorldPos());
        if (!p.allFinite()) continue;
        vPos.push_back(p);
    }
    if (vPos.empty()) return;

    std::vector<Eigen::Vector3f> vClean = FilterOutliersMAD(vPos);
    if (vClean.empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

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

void MonoInertialFileSlamNode::PublishFullMapPointCloud()
{
    if (m_pointcloud_full_publisher->get_subscription_count() == 0) return;

    std::vector<ORB_SLAM3::MapPoint*> vpMPs = m_SLAM->GetAllCurrentMapPoints();
    if (vpMPs.empty()) return;

    std::vector<Eigen::Vector3f> vPos;
    for (auto* pMP : vpMPs) {
        if (!pMP || pMP->isBad()) continue;
        Eigen::Vector3f p = ConvertPointToRos(pMP->GetWorldPos());
        if (!p.allFinite()) continue;
        vPos.push_back(p);
    }
    if (vPos.empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(vPos.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    for (size_t i = 0; i < vPos.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = vPos[i].x();
        *iter_y = vPos[i].y();
        *iter_z = vPos[i].z();
    }

    m_pointcloud_full_publisher->publish(msg);
}

void MonoInertialFileSlamNode::PublishPointCloudCam(const Sophus::SE3f &Tcw)
{
    if (m_pointcloud_cam_publisher->get_subscription_count() == 0) return;

    std::vector<ORB_SLAM3::MapPoint*> vpMPs = m_SLAM->GetTrackedMapPoints();
    if (vpMPs.empty()) return;

    std::vector<Eigen::Vector3f> vPos;
    vPos.reserve(vpMPs.size());
    for (auto* pMP : vpMPs) {
        if (!pMP || pMP->isBad()) continue;
        Eigen::Vector3f p_cam = Tcw * pMP->GetWorldPos();   // world -> kamera
        if (!p_cam.allFinite()) continue;
        vPos.push_back(p_cam);
    }
    if (vPos.empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link_optical";

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(vPos.size());

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    for (size_t i = 0; i < vPos.size(); ++i, ++ix, ++iy, ++iz) {
        *ix = vPos[i].x();
        *iy = vPos[i].y();
        *iz = vPos[i].z();
    }

    m_pointcloud_cam_publisher->publish(msg);
}

// =============================================================================
// Occupancy grid (legacy, dari OccupancyGridBuilder)
// =============================================================================

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