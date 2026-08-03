#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RepubGT : public rclcpp::Node
{
public:
    RepubGT();

    void GTCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void GTPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void Odom2Callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);

private:
    Eigen::Isometry3d T_bw_b0_GT;
    bool isGTInit = false;
    nav_msgs::msg::Path mPath;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mGTPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mGTPosePub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mGTPathPub;

    Eigen::Isometry3d T_bw_b0_odom;
    bool isOdomInit = false;
    nav_msgs::msg::Path mPathOdom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mOdomPosePub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mOdomPathPub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_odom2;
};

RepubGT::RepubGT() : rclcpp::Node("odom_subscriber_node")
{
    // Create a subscriber for the odometry topic
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "/rexrov/pose_gt", rclcpp::QoS(10),
        std::bind(&RepubGT::GTCallback, this, std::placeholders::_1));
    gt_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pose_gt", rclcpp::QoS(10),
        std::bind(&RepubGT::GTPoseCallback, this, std::placeholders::_1));

    mGTPub = create_publisher<nav_msgs::msg::Odometry>("/gt_repub/gt_odom", rclcpp::QoS(10));
    mGTPathPub = create_publisher<nav_msgs::msg::Path>("/gt_repub/gt_path", rclcpp::QoS(10));
    mGTPosePub = create_publisher<geometry_msgs::msg::PoseStamped>("/gt_repub/gt_pose", rclcpp::QoS(10));

    // Create a subscriber for the odometry topic
    odom_sub_odom = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::QoS(10),
        std::bind(&RepubGT::OdomCallback, this, std::placeholders::_1));
    odom_sub_odom2 = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/odom_pose", rclcpp::QoS(10),
        std::bind(&RepubGT::Odom2Callback, this, std::placeholders::_1));

    mOdomPub = create_publisher<nav_msgs::msg::Odometry>("/gt_repub/odom", rclcpp::QoS(10));
    mOdomPathPub = create_publisher<nav_msgs::msg::Path>("/gt_repub/odom_path", rclcpp::QoS(10));
    mOdomPosePub = create_publisher<geometry_msgs::msg::PoseStamped>("/gt_repub/odom_pose", rclcpp::QoS(10));

    mPath.header.stamp = now();
    mPath.header.frame_id = "map";
    mPathOdom.header = mPath.header;
    //init pose
    T_bw_b0_GT.setIdentity();
    T_bw_b0_odom.setIdentity();
}

void RepubGT::GTCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    Eigen::Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Isometry3d T_bw_bj;
    T_bw_bj.setIdentity();
    T_bw_bj.rotate(q);
    T_bw_bj.pretranslate(t);
    if (!isGTInit) {
        T_bw_b0_GT = T_bw_bj;
        isGTInit = true;
        return;
    }
    Eigen::Isometry3d T_b0_bj = T_bw_b0_GT.inverse() * T_bw_bj;
    t = T_b0_bj.translation();
    q = Eigen::Quaterniond(T_b0_bj.rotation());
    nav_msgs::msg::Odometry gt_odom_msg;
    gt_odom_msg.header = msg->header;
    gt_odom_msg.header.frame_id = "map";
    gt_odom_msg.child_frame_id = "base_link";
    gt_odom_msg.pose.pose.position.x = t.x();
    gt_odom_msg.pose.pose.position.y = t.y();
    gt_odom_msg.pose.pose.position.z = t.z();
    gt_odom_msg.pose.pose.orientation.x = q.x();
    gt_odom_msg.pose.pose.orientation.y = q.y();
    gt_odom_msg.pose.pose.orientation.z = q.z();
    gt_odom_msg.pose.pose.orientation.w = q.w();
    mGTPub->publish(gt_odom_msg);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = gt_odom_msg.header;
    pose.pose = gt_odom_msg.pose.pose;
    mGTPosePub->publish(pose);

    mPath.poses.push_back(pose);
    mGTPathPub->publish(mPath);
}

void RepubGT::GTPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
{
    Eigen::Vector3d t(msg->pose.position.y, msg->pose.position.x, msg->pose.position.z);
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Isometry3d T_bw_bj;
    T_bw_bj.setIdentity();
    T_bw_bj.rotate(q);
    T_bw_bj.pretranslate(t);
    if (!isGTInit) {
        T_bw_b0_GT = T_bw_bj;
        isGTInit = true;
        return;
    }
    Eigen::Isometry3d T_b0_bj = T_bw_b0_GT.inverse() * T_bw_bj;
    t = T_b0_bj.translation();
    q = Eigen::Quaterniond(T_b0_bj.rotation());
    nav_msgs::msg::Odometry gt_odom_msg;
    gt_odom_msg.header = msg->header;
    gt_odom_msg.header.frame_id = "map";
    gt_odom_msg.child_frame_id = "base_link";
    gt_odom_msg.pose.pose.position.x = t.x();
    gt_odom_msg.pose.pose.position.y = t.y();
    gt_odom_msg.pose.pose.position.z = t.z();
    gt_odom_msg.pose.pose.orientation.x = q.x();
    gt_odom_msg.pose.pose.orientation.y = q.y();
    gt_odom_msg.pose.pose.orientation.z = q.z();
    gt_odom_msg.pose.pose.orientation.w = q.w();
    mGTPub->publish(gt_odom_msg);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = gt_odom_msg.header;
    pose.pose = gt_odom_msg.pose.pose;
    mGTPosePub->publish(pose);

    mPath.poses.push_back(pose);
    mGTPathPub->publish(mPath);
}

void RepubGT::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    Eigen::Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Isometry3d T_bw_bj;
    T_bw_bj.setIdentity();
    T_bw_bj.rotate(q);
    T_bw_bj.pretranslate(t);
    if (!isOdomInit) {
        T_bw_b0_odom = T_bw_bj;
        isOdomInit = true;
        return;
    }
    Eigen::Isometry3d T_b0_bj = T_bw_b0_odom.inverse() * T_bw_bj;
    t = T_b0_bj.translation();
    q = Eigen::Quaterniond(T_b0_bj.rotation());
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = msg->header;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    mOdomPub->publish(odom_msg);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom_msg.header;
    pose.pose = odom_msg.pose.pose;
    mOdomPosePub->publish(pose);

    mPathOdom.poses.push_back(pose);
    mOdomPathPub->publish(mPathOdom);
}

void RepubGT::Odom2Callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
{
    Eigen::Vector3d t(msg->pose.position.y, msg->pose.position.x, msg->pose.position.z);
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Isometry3d T_bw_bj;
    T_bw_bj.setIdentity();
    T_bw_bj.rotate(q);
    T_bw_bj.pretranslate(t);
    if (!isOdomInit) {
        T_bw_b0_odom = T_bw_bj;
        isOdomInit = true;
        return;
    }
    Eigen::Isometry3d T_b0_bj = T_bw_b0_odom.inverse() * T_bw_bj;
    t = T_b0_bj.translation();
    q = Eigen::Quaterniond(T_b0_bj.rotation());
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = msg->header;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    mOdomPub->publish(odom_msg);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom_msg.header;
    pose.pose = odom_msg.pose.pose;
    mOdomPosePub->publish(pose);

    mPathOdom.poses.push_back(pose);
    mOdomPathPub->publish(mPathOdom);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RepubGT>());
    rclcpp::shutdown();

    return 0;
}

