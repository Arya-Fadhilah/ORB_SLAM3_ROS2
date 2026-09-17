// =============================================================================
// STATUS: Debug/testing tool SEMENTARA — BUKAN pengganti node produksi
// (monocular-inertial-node.cpp topic-based, masih belum ditest).
//
// TUJUAN: pasangan file-based dari MonocularInertialNode (topic-based), sama
// alasannya dengan KittiFileSlamNode — baca dataset langsung dari disk secara
// sekuensial (tanpa topic ROS2 di antara sumber data dan TrackMonocular),
// supaya bisa test pipeline Nav2 untuk mode IMU_MONOCULAR terisolasi dari
// masalah drop-frame komunikasi ROS2.
//
// KENAPA BUKAN FORMAT KITTI:
// Dataset KITTI odometry TIDAK PUNYA data IMU. Node ini pakai pola loading ala
// EuRoC/TUM-VI (gambar + file timestamp + file IMU CSV: t,gx,gy,gz,ax,ay,az)
// mengikuti pola LoadImagesTUMVI()/LoadIMU().
//
// CATATAN DUPLIKASI (sengaja, sementara):
// Publisher point cloud / TF / konversi sumbu di bawah ini adalah SALINAN dari
// KittiFileSlamNode. Duplikasi ini diterima untuk sekarang supaya node KITTI
// yang sudah terbukti jalan tidak ikut diubah. Rencana berikutnya: ekstrak
// jadi satu class bersama (SlamRosPublisher) supaya tidak dirawat dua kali.
//
// PENTING — JANGAN DIPAKAI SEBAGAI NODE PRODUKSI:
// Validasi akhir tetap wajib pakai node topic-based.
// =============================================================================

#ifndef __MONO_INERTIAL_FILE_SLAM_NODE_HPP__
#define __MONO_INERTIAL_FILE_SLAM_NODE_HPP__

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonoInertialFileSlamNode : public rclcpp::Node
{
public:
    MonoInertialFileSlamNode(ORB_SLAM3::System* pSLAM,
                              const std::string& strImagePath,
                              const std::string& strImageTimesFile,
                              const std::string& strImuPath,
                              const std::string& format = "tumvi");

    ~MonoInertialFileSlamNode();

    void RunSequence();

private:
    void PublishOccupancyGrid();

    void LoadImages(const std::string& strImagePath, const std::string& strImageTimesFile,
                     std::vector<std::string>& vstrImages, std::vector<double>& vTimestampsImg);

    void LoadIMU(const std::string& strImuPath,
                 std::vector<double>& vTimestampsImu,
                 std::vector<cv::Point3f>& vAcc,
                 std::vector<cv::Point3f>& vGyro);

    // --- di-port dari KittiFileSlamNode ---
    void PublishPointCloud();
    void PublishFullMapPointCloud();
    void PublishStaticMapToOdom();
    void BroadcastOdomToBaseLink(const Sophus::SE3f &Twc, const rclcpp::Time &stamp);
    void PublishOdometry(const Sophus::SE3f &Twc, const rclcpp::Time &stamp);
    void PublishPointCloudCam(const Sophus::SE3f &Tcw);

    Eigen::Vector3f ConvertPointToRos(const Eigen::Vector3f &p_cam);
    void ConvertPoseToRos(const Sophus::SE3f &Twc_cam,
                          Eigen::Vector3f &trans_ros, Eigen::Quaternionf &q_ros);

    std::vector<Eigen::Vector3f> FilterOutliersMAD(
        const std::vector<Eigen::Vector3f>& pts, float threshold = 15.0f);

    cv::Ptr<cv::CLAHE> m_clahe;
    
    ORB_SLAM3::System* m_SLAM;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_full_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_cam_publisher;


    rclcpp::TimerBase::SharedPtr m_map_timer;
    rclcpp::TimerBase::SharedPtr m_pointcloud_timer;
    rclcpp::TimerBase::SharedPtr m_pointcloud_full_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;

    // State untuk finite-difference /odom
    bool m_has_prev_pose = false;
    Eigen::Vector3f m_prev_trans = Eigen::Vector3f::Zero();
    Eigen::Quaternionf m_prev_q = Eigen::Quaternionf::Identity();
    rclcpp::Time m_prev_stamp;

    std::string m_format;
    std::vector<std::string> m_vstrImages;
    std::vector<double> m_vTimestampsImg;

    std::vector<double> m_vTimestampsImu;
    std::vector<cv::Point3f> m_vAcc;
    std::vector<cv::Point3f> m_vGyro;
};

#endif