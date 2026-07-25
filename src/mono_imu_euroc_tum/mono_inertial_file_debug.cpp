// =============================================================================
// STATUS: BELUM DITEST. Lihat catatan lengkap di mono-inertial-file-slam-node.hpp
//
// Usage: ros2 run orbslam3 mono_inertial_file_debug <path_to_vocabulary> <path_to_settings> <path_to_image_folder> <path_to_image_times_file> <path_to_imu_file>
//
// Format yang diasumsikan (pola TUM-VI):
// - path_to_image_folder: folder berisi file .png dengan nama = timestamp (nanodetik)
// - path_to_image_times_file: file teks, satu timestamp per baris (nanodetik)
// - path_to_imu_file: CSV, format "t,gx,gy,gz,ax,ay,az" per baris (nanodetik)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include "System.h"
#include "mono-inertial-file-slam-node.hpp"

int main(int argc, char** argv)
{
    if (argc != 6)
    {
        std::cerr << "Usage: ros2 run orbslam3 mono_inertial_file_debug "
                  << "<path_to_vocabulary> <path_to_settings> <path_to_image_folder> "
                  << "<path_to_image_times_file> <path_to_imu_file>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

    auto node = std::make_shared<MonoInertialFileSlamNode>(&SLAM, argv[3], argv[4], argv[5]);
    node->RunSequence();

    rclcpp::shutdown();
    return 0;
}
