Terminal 1:

ros2 run orbslam3 mono_inertial_file_debug   
    ~/asv_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt   
    /mnt/e/SLAM_Dataset/ROVER/t264.yaml   
    /mnt/e/SLAM_Dataset/ROVER/realsense_T265/cam_left   
    /mnt/e/SLAM_Dataset/ROVER/realsense_T265/cam_left.txt   
    /mnt/e/SLAM_Dataset/ROVER/realsense_T265/imu/imu.txt   0 rover

cd ~/asv_ws && source install/setup.bash
ros2 run orbslam3 mono_inertial_file_debug \
  ~/asv_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/asv_ws/src/ORB_SLAM3/Examples/Monocular-Inertial/TUM-VI.yaml \
  ~/TUM_VI/dso/cam0/images \
  ~/TUM_VI/dso/cam0/times.txt \
  ~/asv_ws/src/ORB_SLAM3/Examples/Monocular-Inertial/TUM_IMU/dataset-outdoors3_512.txt 0

Terminal 2:

ros2 launch nav2_bringup navigation_launch.py   params_file:=/home/arya_fadhilah/asv_ws/nav2_test/nav2_params_gabungan.yaml

Terminal 3:

rviz2 -d ~/asv_ws/nav2_test/asv_nav2.rviz