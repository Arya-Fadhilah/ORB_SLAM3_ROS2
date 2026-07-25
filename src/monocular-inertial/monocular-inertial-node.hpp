// =============================================================================
// STATUS: BELUM DITEST. Dibuat sesuai permintaan sebagai node baru untuk mode
// IMU_MONOCULAR, karena zang09/ORB_SLAM3_ROS2 tidak menyediakan node ini sama
// sekali (hanya ada: mono, stereo, rgbd, stereo-inertial).
//
// Basis: gabungan pola monocular-slam-node.cpp (callback gambar sederhana)
// + stereo-inertial-node.cpp (buffering/sinkronisasi IMU via thread polling
// manual, bukan message_filters — IMU rate tinggi kurang cocok dengan
// TimeSynchronizer).
//
// PERLU DICEK SEBELUM BUILD:
// 1. Signature ORB_SLAM3::System::TrackMonocular(im, tIm, vImuMeas) di System.h
//    — belum diverifikasi cocok persis dengan versi ORB_SLAM3 di proyek ini.
// 2. Topic "imu" dan "camera" — hardcoded, sesuaikan dengan publisher aktual
//    (driver kamera IMX219-83 / ICM20948) kalau beda nama topic.
//
// CATATAN RISIKO (diwariskan dari eksperimen Integrasi #3):
// Pola GrabImage() di sini "buang frame lama, simpan yang terbaru" — pola
// PERSIS yang terbukti memicu tracking loss & reset berulang (59-160x per run)
// pada eksperimen decoupling monocular murni (tanpa IMU) sebelumnya. Dengan
// IMU, ORB-SLAM3 punya prediksi motion model dari IMU sehingga toleransi gap
// visual KEMUNGKINAN lebih baik — tapi ini BELUM DIBUKTIKAN, murni dugaan.
// Kalau node ini dites dan mengalami reset berulang serupa, ini kandidat
// penyebab pertama yang harus dicek, bukan asumsi baru yang lain.
// =============================================================================

#ifndef __MONOCULAR_INERTIAL_NODE_HPP__
#define __MONOCULAR_INERTIAL_NODE_HPP__

#include <queue>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System* pSLAM);
    ~MonocularInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;

    ORB_SLAM3::System* SLAM_;
    std::thread* syncThread_;

    // IMU
    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image (satu buffer saja — monocular, tidak seperti stereo yang butuh left+right)
    std::queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutexImg_;
};

#endif
