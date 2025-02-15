#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard  # Install using: pip install pynput

class SwerveKeyboardController(Node):
    def __init__(self):
        super().__init__('swerve_keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.speed_step = 0.1  # Increment/decrement step for velocity
        self.max_speed = 1.0   # Maximum speed limit

        self.get_logger().info("Swerve Drive Keyboard Controller Started!")
        self.get_logger().info("Use the following keys to control the robot:")
        self.get_logger().info("  W: Increase linear X velocity (forward)")
        self.get_logger().info("  S: Decrease linear X velocity (backward)")
        self.get_logger().info("  A: Increase linear Y velocity (left)")
        self.get_logger().info("  D: Decrease linear Y velocity (right)")
        self.get_logger().info("  Q: Increase angular Z velocity (counter-clockwise)")
        self.get_logger().info("  E: Decrease angular Z velocity (clockwise)")
        self.get_logger().info("  X: Stop the robot")

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == 'w':  # Increase linear X velocity
                self.linear_x = min(self.linear_x + self.speed_step, self.max_speed)
            elif key.char == 's':  # Decrease linear X velocity
                self.linear_x = max(self.linear_x - self.speed_step, -self.max_speed)
            elif key.char == 'a':  # Increase linear Y velocity
                self.linear_y = min(self.linear_y + self.speed_step, self.max_speed)
            elif key.char == 'd':  # Decrease linear Y velocity
                self.linear_y = max(self.linear_y - self.speed_step, -self.max_speed)
            elif key.char == 'q':  # Increase angular Z velocity
                self.angular_z = min(self.angular_z + self.speed_step, self.max_speed)
            elif key.char == 'e':  # Decrease angular Z velocity
                self.angular_z = max(self.angular_z - self.speed_step, -self.max_speed)
            elif key.char == 'x':  # Stop the robot
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0
        except AttributeError:
            pass  # Ignore special keys

    def on_release(self, key):
        pass  # Do nothing on key release

    def timer_callback(self):
        # Publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_x
        twist_msg.linear.y = self.linear_y
        twist_msg.angular.z = self.angular_z
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SwerveKeyboardController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()