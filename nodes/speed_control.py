#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ds_dbw_msgs.msg import UlcCmd

class SpeedOverrideNode(Node):

    def __init__(self):
        super().__init__('speed_override_node')
        
        # Subscriber to stop command topic
        self.stop_subscriber = self.create_subscription(
            String,
            'stop_cmd',  # Topic name to subscribe to
            self.stop_callback,
            10
        )

        # Publisher to the speed_override topic
        self.speed_publisher = self.create_publisher(UlcCmd, '/speed_override', 10)

        self.timer = None
        self.publish_duration = 5.0  # Duration in seconds for which to publish speed override messages

    def stop_callback(self, msg):
        if msg.data == 'stop':
            self.get_logger().info("Received stop command, starting timer...")
            
            # Create a timer to publish messages at 5 Hz (0.2 seconds)
            self.timer = self.create_timer(0.2, self.publish_speed_override)
            
            # Set up a timer to stop publishing after a specific duration
            self.create_timer(self.publish_duration, self.stop_publishing)

    def publish_speed_override(self):
        msg = UlcCmd()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.cmd = 0.0  # Velcity command in m/s (0.0 for stop)
        msg.limit_decel = 1.0 # Deceleration limit in m/s^2
        msg.cmd_type = UlcCmd.CMD_VELOCITY
        msg.enable = True

        self.speed_publisher.publish(msg)
        self.get_logger().info("Published speed override message")

    def stop_publishing(self):
        if self.timer:
            self.timer.cancel()  # Cancel the publishing timer after the duration
            self.get_logger().info("Stopped publishing speed override messages")


def main(args=None):
    rclpy.init(args=args)
    node = SpeedOverrideNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
