#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import numpy as np
import rospy
import math
from spinal.msg import ServoControlCmd
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop():
    def __init__(self):

        self.joy_dead_zone = rospy.get_param('~joy_dead_zone', 0.1)
        self.vel_rate = rospy.get_param('~vel_rate', 250.0)
        self.max_val = rospy.get_param('~max_val', 250)
        self.keep_vel = rospy.get_param('~keep_vel', 0.0)

        self.fin_angle_rate = rospy.get_param('fin_angle_rate', 1024.0)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twistCallback)

        self.cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)


    def _joyCallback(self, msg):

        forward_vel = msg.axes[1]
        if math.fabs(forward_vel) < self.joy_dead_zone:
            forward_vel = 0

        forward_val = forward_vel * self.vel_rate
        if forward_val > self.max_val:
            forward_val = self.max_val
        if forward_val < -self.max_val:
            forward_val = -self.max_val

        # TODO1
        if msg.buttons[5] == 1: #R2
            forward_val = self.keep_vel
        else:
            self.keep_vel = forward_val

        #TODO2
        fin_angle_ctrl = msg.axes[5]
        fin_angle = fin_angle_ctrl * self.fin_angle_rate + 2048.0
        # TODO1: push L2 or R2 buttons to give constant forward vel

        # TODO2: use right joystick to control the fin

        # motor_vel = np.array([int(forward_vel), int(forward_val)], dtype=np.int16)
        msg_0 = ServoControlCmd()
        msg_0.index = [0,1,2]
        msg_0.cmd.append(int(forward_val))
        msg_0.cmd.append(int(forward_val))
        msg_0.cmd.append(int(fin_angle))
        self.cmd_pub.publish(msg_0)


    def _twistCallback(self, msg):

        vel = msg.linear.x
        val = vel * self.vel_rate
        if vel > self.max_val:
            val = self.max_val
        if vel < -self.max_val:
            val = -self.max_val
        msg = ServoControlCmd()
        msg.index = [0]
        msg.cmd.append(int(val))

        self.cmd_pub.publish(msg)



if __name__=="__main__":

    rospy.init_node("teleop")

    teleop_node = Teleop()

    rospy.spin()
