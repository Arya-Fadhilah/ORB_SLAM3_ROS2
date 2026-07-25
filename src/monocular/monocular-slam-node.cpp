#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        30,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    m_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local());
    m_map_timer = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&MonocularSlamNode::PublishOccupancyGrid, this));

    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SavePointCloud("PointCloud.ply");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    if (m_SLAM->isShutDown())
    {
        rclcpp::shutdown();
        return;
    }

    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}

void MonocularSlamNode::PublishOccupancyGrid()
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