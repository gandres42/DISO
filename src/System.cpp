//
// Created by da on 03/08/23.
//

#include "System.h"
#include "Frame.h"
#include "Track.h"
#include "LocalMapping.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <cstdlib>
#include <string>

System::System(const std::string &strSettingFile)
        : rclcpp::Node("direct_sonar_odometry")
{
    std::string settings = declare_parameter<std::string>("settings_file", strSettingFile);
    if (settings.empty()) { settings = strSettingFile; }
    mOutputDir = declare_parameter<std::string>("output_dir", std::string(""));
    mDebugDir = declare_parameter<std::string>("debug_dir", std::string(""));
    mImageTransport = declare_parameter<std::string>("image_transport", std::string("compressed"));

    if (settings.empty()) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "No settings file given. Pass one as the first command line argument "
                            "or set the 'settings_file' parameter.");
        throw std::runtime_error("direct_sonar_odometry: no settings file given");
    }

    if (!mOutputDir.empty()) {
        std::filesystem::create_directories(mOutputDir);
        RCLCPP_INFO_STREAM(get_logger(), "Output directory: " << mOutputDir);
    }
    if (!mDebugDir.empty()) {
        std::filesystem::create_directories(mDebugDir);
        RCLCPP_INFO_STREAM(get_logger(), "Debug directory: " << mDebugDir);
    }

    cv::FileStorage fsSettings(settings.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to open settings file at: " << settings);
        exit(-1);
    }

    mSonarTopic = (string) fsSettings["SonarTopic"];
    mOdomTopic = (string) fsSettings["OdomTopic"];
    mRange = (double) fsSettings["Range"];
    mGradientThreshold = (double) fsSettings["GradientThreshold"];
    mPyramidLayer = (int) fsSettings["PyramidLayer"];
    mFOV = ((double) fsSettings["FOV"] / 180) * M_PI;
    int loss_threshold = (int) fsSettings["LossThreshold"];
    double gradient_inlier_threshold = (double) fsSettings["GradientInlierThreshold"];
    cv::FileNode use_odom_node = fsSettings["UseOdom"];
    mUseOdom = use_odom_node.empty() ? true : ((int) use_odom_node != 0);
    cv::FileNode node = fsSettings["Tbs"];
    cv::Mat Tbs;
    mT_b_s = Eigen::Isometry3d::Identity();
    if (!node.empty()) {
        Tbs = node.mat();
        if (Tbs.rows != 4 || Tbs.cols != 4) {
            RCLCPP_ERROR_STREAM(get_logger(), "Tbs matrix have to be a 4x4 transformation matrix");
            exit(-1);
        }
    }
    else {
        RCLCPP_ERROR_STREAM(get_logger(), "Tbs matrix doesn't exist");
        exit(-1);
    }
    cv::cv2eigen(Tbs, mT_b_s.matrix());

    RCLCPP_INFO_STREAM(get_logger(), "SonarTopic: " << mSonarTopic);
    RCLCPP_INFO_STREAM(get_logger(), "OdomTopic: " << mOdomTopic);
    RCLCPP_INFO_STREAM(get_logger(), "Range: " << mRange);
    RCLCPP_INFO_STREAM(get_logger(), "GradientThreshold: " << mGradientThreshold);
    RCLCPP_INFO_STREAM(get_logger(), "PyramidLayer: " << mPyramidLayer);
    RCLCPP_INFO_STREAM(get_logger(), "FOV: " << 180 * mFOV / M_PI);
    RCLCPP_INFO_STREAM(get_logger(), "LossThreshold: " << loss_threshold);
    RCLCPP_INFO_STREAM(get_logger(), "GradientInlierThreshold: " << gradient_inlier_threshold);
    RCLCPP_INFO_STREAM(get_logger(), "UseOdom: " << mUseOdom);
    RCLCPP_INFO_STREAM(get_logger(), "Tbs: \n" << fixed << setprecision(9) << Tbs);


    // NOTE: rclcpp::Node declares a static make_shared(), which would hide std::make_shared
    // here, so these have to be explicitly qualified.
    mpTracker = std::make_shared<Track>(this, mRange, mFOV, mPyramidLayer, loss_threshold,
                                   gradient_inlier_threshold, mT_b_s, mUseOdom);
    mpTracker->SetOutputConfig(mOutputDir, mDebugDir);
    shared_ptr<TrackState> track_state = std::make_shared<TrackUpToDate>(mpTracker);
    mpTracker->SetState(track_state);

    mpLocalMaper = std::make_shared<LocalMapping>(this, mpTracker);
    mpLocalMaper->SetDebugDir(mDebugDir);
    mpTracker->SetLocalMaper(mpLocalMaper);

    //start new thread
    mptLocalMaper = std::make_shared<thread>(&LocalMapping::Run, mpLocalMaper);

    mT_bw_b0 = Eigen::Isometry3d::Identity();
    // mOdom_Path.reserve(10000);

    mSonarPosePub = create_publisher<geometry_msgs::msg::PoseStamped>("/direct_sonar/pose_draw",
                                                                     rclcpp::QoS(10));
    mOdomPub = create_publisher<nav_msgs::msg::Odometry>("/direct_sonar/odom_test",
                                                        rclcpp::QoS(1000));


    mPointCloudPub = create_publisher<sensor_msgs::msg::PointCloud2>("/direct_sonar/point_cloud",
                                                                    rclcpp::QoS(1));

    m_tb = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

}

System::~System()
{
    if (mpLocalMaper) { mpLocalMaper->RequestStop(); }
    if (mptLocalMaper && mptLocalMaper->joinable()) { mptLocalMaper->join(); }
    Save(true);
    if (mpTracker) { mpTracker->SavePath(true); }
}

void System::frameLoad(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                       const geometry_msgs::msg::PoseStamped::ConstSharedPtr &odom_msg)
{
    Eigen::Vector3d t(odom_msg->pose.position.y, odom_msg->pose.position.x, odom_msg->pose.position.z);
    Eigen::Quaterniond q(odom_msg->pose.orientation.w, odom_msg->pose.orientation.x,
                         odom_msg->pose.orientation.y, odom_msg->pose.orientation.z);
    double time = rclcpp::Time(image_msg->header.stamp).seconds();
    Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    if (mT_bw_b0.matrix() == I.matrix()) {
        mT_bw_b0.setIdentity();
        mT_bw_b0.rotate(q);
        mT_bw_b0.pretranslate(t);
        mpLocalMaper->mT_bw_b0 = mT_bw_b0;
        // return;
    }
    Eigen::Isometry3d T_bw_bi = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd a(q);
    T_bw_bi.rotate(q);
    T_bw_bi.pretranslate(t);
    Eigen::Isometry3d T_b0_bi = mT_bw_b0.inverse() * T_bw_bi;
    mOdom_Path.insert(make_pair(time, T_b0_bi));
    Save();

    cv::Mat img = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    cv::Mat down;
    cv::pyrDown(img, down, cv::Size(img.cols / 2, img.rows / 2));
    // Frame f(down, rclcpp::Time(image_msg->header.stamp).seconds(), mRange, mFOV, mPyramidLayer, mGradientThreshold);
    Frame f(down, rclcpp::Time(image_msg->header.stamp).seconds(), mRange, mFOV, T_b0_bi, mPyramidLayer,
            mGradientThreshold);
    ExtracPointCloud(down, time, f.mTheta, f.mTx, f.mTy, f.mScale);
    nav_msgs::msg::Odometry odom_test;
    q = Eigen::Quaterniond(T_b0_bi.rotation());
    t = T_b0_bi.translation();
    odom_test.header.stamp = image_msg->header.stamp;
    odom_test.header.frame_id = "odom";
    odom_test.child_frame_id = "base_link";
    odom_test.pose.pose.position.x = t.x();
    odom_test.pose.pose.position.y = t.y();
    odom_test.pose.pose.position.z = t.z();
    odom_test.pose.pose.orientation.x = q.x();
    odom_test.pose.pose.orientation.y = q.y();
    odom_test.pose.pose.orientation.z = q.z();
    odom_test.pose.pose.orientation.w = q.w();
    mOdomPub->publish(odom_test);
    BroadcastTF(T_b0_bi,image_msg->header.stamp,"odom","base_link");
    // mTracker->TrackFrame2Frame(f);
    Eigen::Isometry3d T_s0_si = mpTracker->TrackFrame(f);

    Eigen::Isometry3d T_bw_bi_sonar = mT_bw_b0 * mT_b_s * T_s0_si * mT_b_s.inverse();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";
    pose.pose.position.x = T_bw_bi_sonar.translation().x();
    pose.pose.position.y = T_bw_bi_sonar.translation().y();
    pose.pose.position.z = T_bw_bi_sonar.translation().z();
    q= Eigen::Quaterniond(T_bw_bi_sonar.rotation());
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    mSonarPosePub->publish(pose);

}

void System::frameLoadSonarOnly(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
    double time = rclcpp::Time(image_msg->header.stamp).seconds();
    Eigen::Isometry3d T_b0_bi = Eigen::Isometry3d::Identity();

    cv::Mat img = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    cv::Mat down;
    cv::pyrDown(img, down, cv::Size(img.cols / 2, img.rows / 2));
    Frame f(down, time, mRange, mFOV, T_b0_bi, mPyramidLayer, mGradientThreshold);
    ExtracPointCloud(down, time, f.mTheta, f.mTx, f.mTy, f.mScale);

    Eigen::Isometry3d T_s0_si = mpTracker->TrackFrame(f);
    Eigen::Isometry3d T_bw_bi_sonar = mT_b_s * T_s0_si * mT_b_s.inverse();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";
    pose.pose.position.x = T_bw_bi_sonar.translation().x();
    pose.pose.position.y = T_bw_bi_sonar.translation().y();
    pose.pose.position.z = T_bw_bi_sonar.translation().z();
    Eigen::Quaterniond q(T_bw_bi_sonar.rotation());
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    mSonarPosePub->publish(pose);
}

void System::runRos()
{
    if (mUseOdom) {
        mImageSub.subscribe(*this, mSonarTopic, mImageTransport, rclcpp::QoS(100));
        mOdomSub.subscribe(this, mOdomTopic, rclcpp::QoS(100));
        mSync = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
                MySyncPolicy(10), mImageSub, mOdomSub);
        mSync->registerCallback(std::bind(&System::frameLoad, this,
                                          std::placeholders::_1, std::placeholders::_2));
    }
    else {
        mImageSubOnly = image_transport::create_subscription(
                *this, mSonarTopic,
                std::bind(&System::frameLoadSonarOnly, this, std::placeholders::_1),
                mImageTransport, rclcpp::QoS(100));
    }



    // image_transport::SubscriberFilter image_sub2;
    // image_sub2.subscribe(*this, mSonarTopic, mImageTransport, rclcpp::QoS(100));
    // message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub2;
    // odom_sub2.subscribe(this, mOdomTopic, rclcpp::QoS(100));
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> MySyncPolicy2;
    // auto sync2 = std::make_shared<message_filters::Synchronizer<MySyncPolicy2>>(MySyncPolicy2(10), image_sub2, odom_sub2);
    // sync2->registerCallback(std::bind(&System::frameLoad2, this, std::placeholders::_1, std::placeholders::_2));
    rclcpp::spin(shared_from_this());
}

void System::frameLoad2(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                        const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
{
    cv::Mat img = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    Eigen::Vector3d t(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z);
    Eigen::Quaterniond q(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                         odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    double time = rclcpp::Time(image_msg->header.stamp).seconds();
    Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    if (mT_bw_b0.matrix() == I.matrix()) {
        mT_bw_b0.setIdentity();
        mT_bw_b0.rotate(q);
        mT_bw_b0.pretranslate(t);
        // return;
    }
    Eigen::Isometry3d T_bw_bi = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd a(q);
    T_bw_bi.rotate(q);
    T_bw_bi.pretranslate(t);
    Eigen::Isometry3d T_b0_bi = mT_bw_b0.inverse() * T_bw_bi;
    mOdom_Path.insert(make_pair(time, T_b0_bi));
    Save();
    Frame f(img, rclcpp::Time(image_msg->header.stamp).seconds(), mRange, mFOV, mPyramidLayer,
            mGradientThreshold);
    // mTracker->TrackFrame2Frame(f);
    mpTracker->TrackFrame(f);
}

void System::Save(bool force)
{
    if (mOutputDir.empty()) {
        return;
    }
    if (!force && (++mSaveCounter % 100) != 0) {
        return;
    }
    std::ofstream outfile;
    outfile.open(
            mOutputDir + "/stamped_groundtruth_gt.txt",
            std::ios_base::out);
    outfile << "#timestamp tx ty tz qx qy qz qw" << endl;
    for (auto time_pose: mOdom_Path) {
        Eigen::Isometry3d T_b0_bi = time_pose.second;
        Eigen::Isometry3d T_s0_si = mT_b_s.inverse() * T_b0_bi * mT_b_s;
        // Eigen::Vector3d t = T_s0_si.translation();
        // Eigen::Quaterniond q(T_s0_si.rotation());
        Eigen::Vector3d t = T_b0_bi.translation();
        Eigen::Quaterniond q(T_b0_bi.rotation());
        outfile << fixed << setprecision(12) << time_pose.first << " " << t.x() << " " << t.y() << " " << t.z()
                << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    outfile.close();

}

void System::ExtracPointCloud(const cv::Mat &img, double timestamp, double theta, double tx, double ty,
                              double scale)
{
    cv::Mat gray = img.clone();
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    // Set the intensity threshold
    int threshold = 200; // Change this as required

    // To store points whose intensity is greater than threshold
    std::vector<cv::Point> points;

    // Find points that are greater than threshold
    cv::Mat mask = gray > threshold; // This creates a binary mask
    cv::findNonZero(mask, points);

    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // for (auto p: points) {
    //     Eigen::Vector2d p_pixel(p.x, p.y);
    //     Eigen::Vector3d p_3d = sonar2Dto3D(p_pixel, theta, tx, ty, scale);
    //     pcl::PointXYZ point;
    //     point.x = p_3d(0);
    //     point.y = p_3d(1);
    //     point.z = p_3d(2);
    //
    //     cloud.points.push_back(point);
    //
    // }
    // RCLCPP_INFO_STREAM(get_logger(), "PointCloud Size: "<<cloud.points.size());
    // sensor_msgs::msg::PointCloud2 pc_msg;
    // pcl::toROSMsg(cloud, pc_msg);
    // pc_msg.header.frame_id = "base_link";
    // rclcpp::Time time(static_cast<int64_t>(timestamp * 1e9));
    // pc_msg.header.stamp = time;
    // mPointCloudPub->publish(pc_msg);


}

void System::BroadcastTF(const Eigen::Isometry3d &T_c0_cj_orb,
                              const rclcpp::Time &stamp,
                              const string &id,
                              const string &child_id)
{
    Eigen::Quaterniond rotation_q(T_c0_cj_orb.rotation());
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = id;
    tf_msg.child_frame_id = child_id;
    tf_msg.transform.translation.x = T_c0_cj_orb.translation().x();
    tf_msg.transform.translation.y = T_c0_cj_orb.translation().y();
    tf_msg.transform.translation.z = T_c0_cj_orb.translation().z();
    tf_msg.transform.rotation.x = rotation_q.x();
    tf_msg.transform.rotation.y = rotation_q.y();
    tf_msg.transform.rotation.z = rotation_q.z();
    tf_msg.transform.rotation.w = rotation_q.w();
    m_tb->sendTransform(tf_msg);
}
