#!/usr/bin/python3
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import threading
import select
import termios
import tty

vel_msg = Twist() #A global Twist message that stores the robot's current velocity (linear and angular).
use_joystick = False  # Flag for joystick control
use_external_cmd_vel = False  # Flag for external cmd_vel control

class Commander(Node): #node that publishes velocity commands to the /helios/cmd_vel
    def __init__(self):
        super().__init__('commander')
        self.publisher_ = self.create_publisher(Twist, '/helios/cmd_vel', 10) 
        timer_period = 0.02 #triggered every 0.02 seconds to publish the current velocity (vel_msg).
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg
        self.publisher_.publish(vel_msg)

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

    def listener_callback(self, data):
        global vel_msg, use_joystick, use_external_cmd_vel
        use_joystick = True  # Joystick input takes precedence
        use_external_cmd_vel = False  # Disable external cmd_vel when joystick is used
        vel_msg.linear.x = data.axes[1]
        vel_msg.angular.z = data.axes[0]

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, data):
        global vel_msg, use_external_cmd_vel
        use_external_cmd_vel = True  # External cmd_vel input takes precedence
        vel_msg = data

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    global vel_msg, use_joystick, use_external_cmd_vel
    rclpy.init(args=args)

    commander = Commander()
    joy_subscriber = JoySubscriber()
    cmd_vel_subscriber = CmdVelSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)
    executor.add_node(cmd_vel_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    interactive_mode = sys.stdin.isatty()  # Check if the script is connected to a terminal

    if interactive_mode:
        settings = termios.tcgetattr(sys.stdin)

    try:
        while True:
            if interactive_mode:
                key = get_key(settings)
                if not use_joystick and not use_external_cmd_vel:  # Only use keyboard input if no joystick or external cmd_vel input is active
                    if key == 'w':
                        vel_msg.linear.x = 0.5
                        vel_msg.angular.z = 0.0
                    elif key == 's':
                        vel_msg.linear.x = -0.5
                        vel_msg.angular.z = 0.0
                    elif key == 'a':
                        vel_msg.linear.x = 0.0
                        vel_msg.angular.z = 0.5
                    elif key == 'd':
                        vel_msg.linear.x = 0.0
                        vel_msg.angular.z = -0.5
                    elif key == ' ':
                        vel_msg.linear.x = 0.0
                        vel_msg.angular.z = 0.0
                    elif key == '\x03':  # Ctrl+C
                        break

            # Reset joystick and external cmd_vel use flags after a small timeout
            if use_joystick:
                rclpy.spin_once(joy_subscriber, timeout_sec=0.1)
                use_joystick = False

            if use_external_cmd_vel:
                rclpy.spin_once(cmd_vel_subscriber, timeout_sec=0.1)
                use_external_cmd_vel = False

    except Exception as e:
        print(e)

    finally:
        vel_msg = Twist()
        commander.destroy_node()
        joy_subscriber.destroy_node()
        cmd_vel_subscriber.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

        if interactive_mode:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
