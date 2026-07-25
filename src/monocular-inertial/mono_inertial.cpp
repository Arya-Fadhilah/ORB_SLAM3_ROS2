// =============================================================================
// STATUS: BELUM DITEST. Lihat catatan lengkap di monocular-inertial-node.hpp.
//
// Usage: ros2 run orbslam3 mono_inertial path_to_vocabulary path_to_settings
//
// PENTING sebelum build: pastikan signature
//   ORB_SLAM3::System::TrackMonocular(const cv::Mat&, double, vector<IMU::Point>&)
// benar-benar ada di System.h versi ORB_SLAM3 yang dipakai proyek ini.
// =============================================================================

#include <iostream>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-node.hpp"
#include "System.h"

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 mono_inertial path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    auto node = std::make_shared<MonocularInertialNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
