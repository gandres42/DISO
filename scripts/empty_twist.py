import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ZeroVelPublisher(Node):
    def __init__(self):
        # Initialize the node with a specific name
        super().__init__('zero_vel_publisher')

        # Create a publisher on the 'cmd_vel' topic with a queue size of 10
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # 10 Hz frequency means a period of 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Publishing zero velocity to cmd_vel at 10Hz...')

    def timer_callback(self):
        msg = TwistStamped()

        # cmd_vel_odom copies this onto the PoseStamped it publishes, and that is what
        # gets matched against the sonar image.  Left unset it stays 0 and nothing
        # ever synchronizes.  Run with use_sim_time:=true against a bag played with
        # --clock so this is bag time, not wall time.
        msg.header.stamp = self.get_clock().now().to_msg()

        # In ROS2 Python, Twist() initializes all fields to 0.0 by default,
        # but they are set explicitly here for clarity and safety.
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    zero_vel_publisher = ZeroVelPublisher()

    try:
        rclpy.spin(zero_vel_publisher)
    except KeyboardInterrupt:
        # Allow clean exit via Ctrl+C without throwing an error trace
        pass
    finally:
        # Destroy the node explicitly
        zero_vel_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
