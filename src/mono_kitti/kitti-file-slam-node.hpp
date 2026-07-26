// =============================================================================
// STATUS: Debug/testing tool SEMENTARA — BUKAN pengganti node produksi.
//
// TUJUAN: baca sequence gambar KITTI langsung dari path di disk (bukan lewat
// topic ROS2 "/camera"), panggil TrackMonocular() secara sekuensial, sambil
// tetap publish OccupancyGrid ke topic "/map" seperti node produksi — supaya
// pipeline Nav2 (costmap, planner, dst) bisa ditest tanpa terganggu masalah
// drop-frame komunikasi ROS2 (lihat catatan di bawah).
//
// KENAPA NODE INI ADA (konteks dari Integrasi #3):
// Eksperimen mengukur drop-frame antara publisher (ros2 bag play) dan
// subscriber (monocular-slam-node.cpp GrabImage -> TrackMonocular) menunjukkan
// GrabImage() itu BLOCKING SYNCHRONOUS: TrackMonocular() dipanggil langsung
// di callback subscriber, jadi kalau processing satu frame lebih lambat dari
// interval publish, frame berikutnya menumpuk di queue DDS sampai penuh lalu
// di-drop (terukur: drop 1.4%-47% tergantung run, TIDAK konsisten/tidak
// prediktif dari run ke run).
//
// Sempat dicoba fix "decoupling" (GrabImage cuma simpan ke buffer, thread
// terpisah proses "ambil yang terbaru, buang yang lama") — hasilnya JAUH LEBIH
// BURUK untuk monocular VSLAM: reset/tracking-loss naik drastis (dari ~1-2
// maps jadi 7-22 maps per run), karena continuity visual antar-frame yang
// diproses jadi berlubang secara tidak terprediksi, meski gap waktunya kecil.
// Decoupling ini DIBATALKAN, kembali ke versi blocking yang asli.
//
// Kesimpulannya: drop-frame di level komunikasi ROS2 itu murni masalah
// arsitektur komunikasi, TIDAK ADA HUBUNGANNYA dengan validitas pipeline
// Nav2/OccupancyGridBuilder itu sendiri. Node ini menghilangkan variabel
// drop-frame itu sepenuhnya (baca file langsung, sekuensial, tidak ada
// queue/topic di antara sumber gambar dan TrackMonocular) supaya testing
// Nav2 pipeline bisa dilakukan terisolasi dari masalah itu.
//
// PENTING — JANGAN DIPAKAI SEBAGAI NODE PRODUKSI:
// Di ASV real, sumber gambar adalah kamera live yang WAJIB lewat topic ROS2
// standar (sensor_msgs::Image) supaya bisa terhubung ke node lain. Node ini
// TIDAK merepresentasikan itu. Validasi akhir skripsi (klaim novelty: full
// Nav2 stack tervalidasi di ASV real via ORB-SLAM3) tetap WAJIB pakai node
// topic-based (monocular-slam-node.cpp/mono.cpp yang asli, tidak diubah).
// =============================================================================

#ifndef __KITTI_FILE_SLAM_NODE_HPP__
#define __KITTI_FILE_SLAM_NODE_HPP__

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class KittiFileSlamNode : public rclcpp::Node
{
public:
    KittiFileSlamNode(ORB_SLAM3::System* pSLAM, const std::string& strSequencePath, const std::string& strTimesFile);

    ~KittiFileSlamNode();

    // Dipanggil manual dari main() setelah node dibuat — bukan callback,
    // karena tidak ada subscriber yang perlu di-spin menunggu data masuk.
    void RunSequence();

private:
    void PublishOccupancyGrid();
    void LoadImages(const std::string& strSequencePath, const std::string& strTimesFile,
                     std::vector<std::string>& vstrImageFilenames, std::vector<double>& vTimestamps);

    void PublishPointCloud();
    std::vector<Eigen::Vector3f> FilterOutliersMAD(
        const std::vector<Eigen::Vector3f>& pts, float threshold = 15.0f);
    
    ORB_SLAM3::System* m_SLAM;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
    rclcpp::TimerBase::SharedPtr m_map_timer;
    rclcpp::TimerBase::SharedPtr m_pointcloud_timer;

    std::vector<std::string> m_vstrImageFilenames;
    std::vector<double> m_vTimestamps;
};

#endif
