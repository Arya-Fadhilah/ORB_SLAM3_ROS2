// =============================================================================
// STATUS: Debug/testing tool SEMENTARA. Lihat catatan lengkap di
// kitti-file-slam-node.hpp sebelum menganggap ini pengganti node produksi.
//
// Usage: ros2 run orbslam3 kitti_file_debug <path_to_vocabulary> <path_to_settings> <path_to_sequence> <path_to_times_file>
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include "System.h"
#include "kitti-file-slam-node.hpp"

int main(int argc, char** argv)
{
    if (argc != 5)
    {
        std::cerr << "Usage: ros2 run orbslam3 kitti_file_debug <path_to_vocabulary> <path_to_settings> <path_to_sequence> <path_to_times_file>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    //bool bUseViewer = true;
    //if (argc >= 6) bUseViewer = (std::string(argv[5]) != "0");

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    auto node = std::make_shared<KittiFileSlamNode>(&SLAM, argv[3], argv[4]);
    node->RunSequence();

    rclcpp::shutdown();
    return 0;
}
