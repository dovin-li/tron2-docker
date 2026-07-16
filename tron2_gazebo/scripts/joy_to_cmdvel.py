#!/usr/bin/env python3
"""Xbox 360 joystick via ROS joy_node → /cmd_vel (like TRON1)"""
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoyToCmdVel:
    def __init__(self):
        rospy.init_node('joy_to_cmdvel')
        self.mode_pub = rospy.Publisher('/tron2_controller/set_mode', String, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.cb)
        rospy.loginfo('Joystick ready — L1+Y:WALK, L1+X:STOP')

    def cb(self, msg):
        b = msg.buttons
        l1 = b[4] if len(b) > 4 else 0
        y_b = b[3] if len(b) > 3 else 0
        x_b = b[2] if len(b) > 2 else 0
        if l1 and y_b: self.mode_pub.publish(String("WALK"))
        if l1 and x_b: self.mode_pub.publish(String("IDLE"))

        ax = msg.axes
        vx = ax[1] * 0.3 if len(ax) > 1 else 0
        vy = ax[0] * 0.2 if len(ax) > 0 else 0
        wz = ax[3] * 0.5 if len(ax) > 3 else 0
        if abs(vx) < 0.05: vx = 0
        if abs(vy) < 0.05: vy = 0
        if abs(wz) < 0.1: wz = 0

        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = -vx, -vy, -wz
        self.cmd_pub.publish(t)

if __name__ == '__main__':
    JoyToCmdVel()
    rospy.spin()
