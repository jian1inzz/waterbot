#!/usr/bin/env python3
# coding=utf-8

import os
import select
import sys
import termios
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

b : OmniMode ON/OFF
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

# 初始速度
speed = 0.2
turn = 0.3

# 最大速度限制（單位：m/s 與 rad/s）
MAX_LINEAR_SPEED = 3.0
MAX_ANGULAR_SPEED = 0.3

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('keyboard_control')
    pub = node.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))

    omni_mode = False
    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings)
            twist = Twist()

            if key == 'b':
                omni_mode = not omni_mode
                mode = "OmniMode" if omni_mode else "CommonMode"
                print(f"Switched to {mode}")
                moveBindings['.'] = (-1, -1) if omni_mode else (-1, 1)
                moveBindings['m'] = (-1, 1) if omni_mode else (-1, -1)

            elif key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][1]

                vx = speed * x
                vy = speed * th if omni_mode else 0.0
                wz = 0.0 if omni_mode else turn * th

                # 加入速度限制，防止超過 int16_t 範圍
                vx = max(min(vx, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
                vy = max(min(vy, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
                wz = max(min(wz, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

                twist.linear.x = vx
                twist.linear.y = vy
                twist.angular.z = wz
                pub.publish(twist)

            else:
                # 放開鍵時送停止訊號
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)

            if key == '\x03':  # Ctrl+C
                break

    except Exception as e:
        print(f"Error: {e}")

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
