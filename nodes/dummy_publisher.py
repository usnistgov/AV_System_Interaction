#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ds_dbw_msgs.msg import UlcCmd
from pynput import keyboard

class DummyNode(Node):

    def __init__(self):
        super().__init__('dummy_node')

        # Subscriber to speed_override topic
        self.speed_subscriber = self.create_subscription(
            UlcCmd, 
            '/speed_override', 
            self.speed_override_callback, 
            10
        )

        # Publisher to stop_cmd topic
        self.stop_publisher = self.create_publisher(String, 'stop_cmd', 10)

        # Start keyboard listener using pynput
        self.keyboard_listener()

    def speed_override_callback(self, msg):
        self.get_logger().info(f"Received speed_override message: {msg}")

    def keyboard_listener(self):
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.start()

    def on_key_press(self, key):
        try:
            if key == keyboard.Key.space:
                self.send_stop_command()
        except Exception as e:
            self.get_logger().error(f"Error in keyboard listener: {e}")

    def send_stop_command(self):
        self.get_logger().info("Space key pressed, sending stop command...")
        stop_msg = String()
        stop_msg.data = 'stop'
        self.stop_publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DummyNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
