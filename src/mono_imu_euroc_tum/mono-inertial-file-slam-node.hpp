// =============================================================================
// STATUS: BELUM DITEST. Debug/testing tool SEMENTARA — BUKAN pengganti node
// produksi (monocular-inertial-node.cpp topic-based, juga belum ditest).
//
// TUJUAN: pasangan file-based dari MonocularInertialNode (topic-based), sama
// alasannya dengan KittiFileSlamNode — baca dataset langsung dari disk secara
// sekuensial (tanpa topic ROS2 di antara sumber data dan TrackMonocular),
// supaya bisa test pipeline Nav2 untuk mode IMU_MONOCULAR terisolasi dari
// masalah drop-frame komunikasi ROS2.
//
// KENAPA BUKAN FORMAT KITTI:
// Dataset KITTI odometry (00-02, yang dipakai sepanjang sesi Integrasi #3)
// TIDAK PUNYA data IMU. Node ini pakai pola loading ala EuRoC/TUM-VI (gambar
// + file timestamp + file IMU terpisah dengan format CSV: t,gx,gy,gz,ax,ay,az)
// mengikuti pola LoadImagesTUMVI()/LoadIMU() yang sudah ada di
// Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc — SESUAIKAN kalau dataset
// IMU yang mau kamu pakai formatnya beda.
//
// PENTING — JANGAN DIPAKAI SEBAGAI NODE PRODUKSI:
// Sama seperti KittiFileSlamNode, ini murni alat debug offline. Validasi akhir
// tetap wajib pakai node topic-based (monocular-inertial-node.cpp), yang juga
// masih berstatus belum ditest sama sekali.
// =============================================================================

#ifndef __MONO_INERTIAL_FILE_SLAM_NODE_HPP__
#define __MONO_INERTIAL_FILE_SLAM_NODE_HPP__

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
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
                              const std::string& strImuPath);

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

    ORB_SLAM3::System* m_SLAM;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_publisher;
    rclcpp::TimerBase::SharedPtr m_map_timer;

    std::vector<std::string> m_vstrImages;
    std::vector<double> m_vTimestampsImg;

    std::vector<double> m_vTimestampsImu;
    std::vector<cv::Point3f> m_vAcc;
    std::vector<cv::Point3f> m_vGyro;
};

#endif
