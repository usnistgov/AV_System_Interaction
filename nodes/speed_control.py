#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ds_dbw_msgs.msg import UlcCmd
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult

class SpeedOverrideNode(Node):

    def __init__(self):
        super().__init__('speed_override_node')

        self.publisher_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_group = MutuallyExclusiveCallbackGroup()
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameter('publish_duration', 8.0)
        self.publish_duration = self.get_parameter('publish_duration').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.stop_subscriber = self.create_subscription(
            String,
            'stop_cmd',
            self.stop_callback,
            qos_profile,
            callback_group=self.subscriber_group
        )

        self.speed_publisher = self.create_publisher(UlcCmd, '/speed_override', qos_profile,
                                                     callback_group=self.publisher_group)

        self.timer = None
        self.remaining_time = 0.0

    def stop_callback(self, msg):
        if msg.data.lower() == 'stop':
            self.get_logger().info("Received stop command, starting timer...")

            if self.timer is not None:
                if self.timer.is_canceled():
                    self.get_logger().warn("Timer was already canceled, skipping cancel operation.")
                else:
                    self.timer.cancel()

            self.remaining_time = self.publish_duration

            # Start the publishing timer at 5 Hz (0.2 seconds interval)
            self.timer = self.create_timer(0.2, self.publish_speed_override)
        else:
            self.get_logger().warn(f"Received unknown command: {msg.data}")

    def publish_speed_override(self):
        if self.remaining_time > 0:
            msg = UlcCmd()
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.cmd = 0.0  # Velocity command in m/s (0.0 for stop)
            msg.limit_decel = 1.5  # Deceleration limit in m/s^2
            msg.cmd_type = UlcCmd.CMD_VELOCITY
            msg.enable = True

            self.speed_publisher.publish(msg)
            self.get_logger().debug(f"Published speed override message, {self.remaining_time:.1f} seconds remaining")

            self.remaining_time -= 0.2
        else:
            self.get_logger().info("Stopping speed override, timer expired.")
            if self.timer is not None:
                self.timer.cancel()  # Cancel the publishing timer after the duration
            self.timer = None
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'publish_duration' and param.type_ == rclpy.Parameter.Type.DOUBLE:
                self.publish_duration = param.value
                self.get_logger().info(f"Updated publish_duration to {param.value} seconds")
        return SetParametersResult(successful=True)

    def on_shutdown(self):
        if self.timer is not None and not self.timer.is_canceled():
            self.timer.cancel()

def main(args=None):

    rclpy.init(args=args)
    node = SpeedOverrideNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        executor.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()