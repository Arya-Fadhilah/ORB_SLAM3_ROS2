// =============================================================================
// STATUS: BELUM DITEST. Lihat catatan lengkap di monocular-inertial-node.hpp.
//
// FIX YANG SUDAH DITERAPKAN dibanding stereo-inertial-node.cpp aslinya:
// - Bug null-pointer-dereference di GetImage(): kode asli tidak `return` di
//   dalam blok catch(cv_bridge::Exception&), sehingga lanjut mengakses
//   cv_ptr->image yang masih default-constructed (null) kalau konversi gagal.
//   Di sini sudah ditambah `return cv::Mat();` + guard `if (im.empty())` di
//   pemanggil (SyncWithImu) untuk skip frame yang gagal, bukan crash.
// =============================================================================

#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_MONOCULAR_INERTIAL_ROS2")
{
    SLAM_ = pSLAM;

    subImu_ = this->create_subscription<ImuMsg>(
        "imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>(
        "camera", 100, std::bind(&MonocularInertialNode::GrabImage, this, _1));

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);

    std::cout << "MonocularInertialNode initialized" << std::endl;
}

MonocularInertialNode::~MonocularInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM_->SavePointCloud("PointCloud.ply");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();

    // Pola "buang yang lama, simpan yang terbaru" — lihat catatan risiko
    // di header file sebelum menganggap ini pola yang aman untuk semua kasus.
    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msg);

    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();  // FIX: return early, jangan lanjut akses cv_ptr->image (null)
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialNode::SyncWithImu()
{
    while (1)
    {
        cv::Mat im;
        double tIm = 0;

        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            bufMutexImg_.lock();
            tIm = Utility::StampToSec(imgBuf_.front()->header.stamp);
            bufMutexImg_.unlock();

            bufMutex_.lock();
            if (imuBuf_.empty() || tIm > Utility::StampToSec(imuBuf_.back()->header.stamp))
            {
                bufMutex_.unlock();
                continue;
            }
            bufMutex_.unlock();

            bufMutexImg_.lock();
            im = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();

            if (im.empty())  // guard tambahan: kalau GetImage gagal (fix di atas), skip frame ini
            {
                continue;
            }

            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x,
                                    imuBuf_.front()->linear_acceleration.y,
                                    imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x,
                                    imuBuf_.front()->angular_velocity.y,
                                    imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            SLAM_->TrackMonocular(im, tIm, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
