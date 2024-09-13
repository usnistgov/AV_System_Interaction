#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ds_dbw_msgs.msg import UlcCmd
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class SpeedOverrideNode(Node):

    def __init__(self):
        super().__init__('speed_override_node')

        self.publisher_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_group = MutuallyExclusiveCallbackGroup()
        
        # Subscriber to stop command topic
        self.stop_subscriber = self.create_subscription(
            String,
            'stop_cmd',  # Topic name to subscribe to
            self.stop_callback,
            10,
            callback_group=self.subscriber_group
        )

        # Publisher to the speed_override topic
        self.speed_publisher = self.create_publisher(UlcCmd, '/speed_override', 10,
                                                     callback_group=self.publisher_group)

        self.timer = None
        self.publish_duration = 8.0  # Duration in seconds for which to publish speed override messages
        self.remaining_time = 0.0


    def stop_callback(self, msg):
        if msg.data == 'stop':
            self.get_logger().info("Received stop command, starting timer...")

            if self.timer is not None:
                self.timer.cancel()

            self.remaining_time = self.publish_duration
            
            # Create a timer to publish messages at 5 Hz (0.2 seconds)
            self.timer = self.create_timer(0.2, self.publish_speed_override)


    def publish_speed_override(self):
        if self.remaining_time > 0:
            msg = UlcCmd()
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.cmd = 0.0  # Velcity command in m/s (0.0 for stop)
            msg.limit_decel = 1.5 # Deceleration limit in m/s^2
            msg.cmd_type = UlcCmd.CMD_VELOCITY
            msg.enable = True

            self.speed_publisher.publish(msg)
            self.get_logger().info("Published speed override message")

            self.remaining_time -= 0.2
        else:
            self.timer.cancel()  # Cancel the publishing timer after the duration
            self.timer = None


def main(args=None):

    rclpy.init(args=args) 
    node = SpeedOverrideNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
