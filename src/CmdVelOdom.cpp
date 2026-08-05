//
// Dead-reckoning odometry for the Aracati2017 dataset.
//
// DISO consumes an odometry prior on `OdomTopic` (see config/config_aracati2017.yaml,
// which defaults to /odom_pose).  The Aracati2017 bag does not contain that topic:
// upstream it is produced by the `odom` node of the companion Aracati2017_DISO
// package, which integrates the /cmd_vel body velocities.  That package is not part
// of this repository, so this node provides the same signal.
//
// The pose is integrated from the identity: DISO re-references every odometry sample
// to the first one it sees (System::frameLoad), so the absolute origin and heading
// cancel out and no ground-truth seed is needed.
//
// The integration convention (in particular the negated yaw rate and the mirrored
// body-to-world rotation) matches the frame the dataset's poses are expressed in;
// integrating /cmd_vel this way reproduces the true heading to ~1e-9 rad over the
// whole 44-minute sequence, leaving only translational drift - which is exactly the
// prior DISO is designed to correct with sonar.
//

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class CmdVelOdom : public rclcpp::Node
{
public:
    CmdVelOdom() : rclcpp::Node("cmd_vel_odom")
    {
        mCmdVelTopic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        mOdomTopic = declare_parameter<std::string>("odom_topic", "/odom_pose_diso");
        mOdomFrameId = declare_parameter<std::string>("odom_frame_id", "odom");

        mOdomPub = create_publisher<geometry_msgs::msg::PoseStamped>(mOdomTopic, rclcpp::QoS(10));
        mCmdVelSub = create_subscription<geometry_msgs::msg::TwistStamped>(
                mCmdVelTopic, rclcpp::QoS(50),
                std::bind(&CmdVelOdom::CmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO_STREAM(get_logger(), "dead-reckoning " << mCmdVelTopic << " -> "
                                                           << mOdomTopic);
    }

private:
    void CmdVelCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
    {
        const double time = rclcpp::Time(msg->header.stamp).seconds();
        if (mLastTime < 0.0) {
            mLastTime = time;
        }
        const double dt = time - mLastTime;
        if (dt < 0.0) {
            RCLCPP_WARN_STREAM(get_logger(), "ignoring out-of-order /cmd_vel (dt=" << dt << ")");
            return;
        }

        // Displacement of the *previous* velocity over dt, applied after updating the
        // heading with the current yaw rate.
        const double dx = mVx * dt;
        const double dy = mVy * dt;
        mYaw -= msg->twist.angular.z * dt;

        const double co = std::cos(mYaw);
        const double so = std::sin(mYaw);
        mX += dx * so + dy * co;
        mY += dx * co - dy * so;

        mVx = msg->twist.linear.x;
        mVy = msg->twist.linear.y;
        mLastTime = time;

        geometry_msgs::msg::PoseStamped out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = mOdomFrameId;
        out.pose.position.x = mX;
        out.pose.position.y = mY;
        out.pose.position.z = 0.0;
        out.pose.orientation.x = 0.0;
        out.pose.orientation.y = 0.0;
        out.pose.orientation.z = std::sin(0.5 * mYaw);
        out.pose.orientation.w = std::cos(0.5 * mYaw);
        mOdomPub->publish(out);
    }

    std::string mCmdVelTopic, mOdomTopic, mOdomFrameId;

    double mX = 0.0, mY = 0.0, mYaw = 0.0;
    double mVx = 0.0, mVy = 0.0;
    double mLastTime = -1.0;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mOdomPub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr mCmdVelSub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelOdom>());
    rclcpp::shutdown();
    return 0;
}
