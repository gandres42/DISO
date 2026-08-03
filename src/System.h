//
// Created by da on 03/08/23.
//

#ifndef SRC_SYSTEM_H
#define SRC_SYSTEM_H

#include<string>
#include <thread>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

using namespace std;

class Track;
class LocalMapping;
class System : public rclcpp::Node
{
public:
    explicit System(const std::string &strSettingFile);

    ~System();

    void frameLoad(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                   const geometry_msgs::msg::PoseStamped::ConstSharedPtr &odom_msg);

    void frameLoad2(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);

    void ExtracPointCloud(const cv::Mat &img, double timestamp, double theta, double tx, double ty,
                          double scale);
    void BroadcastTF(const Eigen::Isometry3d &T_c0_cj_orb,
                             const rclcpp::Time &stamp,
                             const string &id,
                             const string &child_id);
    void Save(bool force = false);

    void runRos();

private:
    string mSonarTopic;
    string mOdomTopic;
    double mRange;
    double mGradientThreshold;
    int mPyramidLayer;
    double mFOV;

    std::string mOutputDir, mDebugDir, mImageTransport;
    int mSaveCounter = 0;

    //odometry initial pose
    Eigen::Isometry3d mT_bw_b0;
    //extrinsic parameter sonar to body
    Eigen::Isometry3d mT_b_s;
    map<double, Eigen::Isometry3d> mOdom_Path;
    shared_ptr<Track> mpTracker;
    shared_ptr<LocalMapping> mpLocalMaper;
    shared_ptr<thread> mptLocalMaper;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mSonarPosePub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tb;
    // ros::Publisher mOdomPub;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped> MySyncPolicy;
    image_transport::SubscriberFilter mImageSub;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> mOdomSub;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> mSync;
};


#endif //SRC_SYSTEM_H
