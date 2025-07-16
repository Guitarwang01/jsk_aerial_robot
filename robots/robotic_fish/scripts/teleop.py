#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import numpy as np
import rospy
import math

from duplicity.commandline import select_files
from hgext.fsmonitor.pywatchman import sniff_len
from spinal.msg import ServoControlCmd
from spinal.msg import ServoStates
from spinal.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop():
    def __init__(self):

        self.joy_dead_zone = rospy.get_param('~joy_dead_zone', 0.1)
        self.vel_rate = rospy.get_param('~vel_rate', 250.0)
        self.max_val = rospy.get_param('~max_val', 250)
        self.max_turn_angle_val = rospy.get_param('~max_turn_angle', 1024)
        self.keep_vel = rospy.get_param('~keep_vel', [0.0, 0.0])
        self.servo_equ_angles = rospy.get_param('~servos_angle', [0, 0, 0])
        self.servo_cmd = rospy.get_param('~servo_cmd', [0, 0, 1024.0])
        self.turning = rospy.get_param('~turning', False)
        self.imu_angle = rospy.get_param('imu_angle', 0.0)
        self.tar_angle = rospy.get_param('~tar_angle', 0.0)
        self.kp = rospy.get_param('~kp', 0.0)

        self.fin_angle_rate = rospy.get_param('fin_angle_rate', 1024.0)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twistCallback)
        self.servo_pos_sub = rospy.Subscriber('/servo/states', ServoStates, self._servoStateCallback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self._imuCallback)

        self.cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)


    def _servoStateCallback(self, msg):
        self.servo_equ_angles = [msg.servos[0].angle % 4096, msg.servos[1].angle % 4096, msg.servos[2].angle]
        #rospy.loginfo("servo 0 at angle %s", msg.servos[0].angle)

    def _imuCallback(self, msg):
        self.imu_angle = msg.angles[2]
        # rospy.loginfo(self.imu_angle)

    def set_tar_angle(self):
        self.tar_angle = self.imu_angle
        rospy.loginfo(self.imu_angle)

    def imu_feedback(self, target_angle):
        # 22 beats per round, such that gentle mode rotates at 16.3 deg/s, 0.286 rad/s
        diff = target_angle - self.imu_angle
        if math.fabs(diff) > math.pi:
            if diff > 0:
                diff -= math.pi
            else:
                diff += math.pi
        if diff == 0:
            kp = 0
        elif math.fabs(diff) > 0.286:
            if diff > 0:
                kp = 1
            else:
                kp = -1
        else:
            kp = diff/0.286

        if math.fabs(self.servo_equ_angles[1] - 2048) < 100:
            self.kp = kp
        self.servo_cmd[0] = self.pos_control(0, 2048 + 1024 * self.kp) #direction?

    def pos_control(self, servo_no, tar):
        pos = self.servo_equ_angles[servo_no]
        if math.fabs(tar - pos) > 2048:
            if tar-pos < 0:
                tar += 4096
            else:
                tar -= 4096
        if tar-pos < 0:
            direction = -1
        else:
            direction = 1

        kp = 2.8
        if math.fabs(tar-pos) > 700:
            return direction*self.max_val
        if math.fabs(tar-pos) > 50:
            # if (tar-pos)/2048 * kp >= 1:
            #     return direction*self.max_val
            return (tar-pos)/2048*kp*self.max_val
        else:
            return 0.0

    def servo_sync(self):
        self.servo_cmd[0] = self.pos_control(0, 2048)
        self.servo_cmd[1] = self.pos_control(1, 2048)

    def left_pose(self):
        self.servo_cmd[0] = self.pos_control(0, 1024)
        self.servo_cmd[1] = self.pos_control(1, 3072)

    def right_pose(self):
        self.servo_cmd[0] = self.pos_control(0, 3072)
        self.servo_cmd[1] = self.pos_control(1, 1024)

    def turn_mode(self):
        if math.fabs(self.servo_equ_angles[0] -2048) > self.max_turn_angle_val + 50:
            return
        else:
            self.turning = True

    def _joyCallback(self, msg):

        # forward
        forward_vel = msg.axes[1]
        if math.fabs(forward_vel) < self.joy_dead_zone:
            forward_vel = 0

        forward_val = forward_vel * self.vel_rate
        if forward_val > self.max_val:
            forward_val = self.max_val
        if forward_val < -self.max_val:
            forward_val = -self.max_val

        self.servo_cmd[0] = -forward_val
        self.servo_cmd[1] = forward_val

        #continuous turning
        if msg.buttons[4] == 1:
            self.turn_mode()
        else:
            self.turning = False
        turn_angle = 2048 + msg.axes[2]*self.max_turn_angle_val
        if self.turning:
            self.servo_cmd[0] = self.pos_control(0, turn_angle)

        # if (math.fabs(self.servo_equ_angles[1] - 2048) > self.max_turn_angle_val and
        #         math.fabs(self.servo_equ_angles[1]-self.servo_equ_angles[0]) < 100):
        #     self.servo_cmd[1] = forward_val

        # hold servo 1 and only control servo 0
        # if msg.buttons[0] == 1: # square
        #     self.servo_cmd[1] = 0

        # only control one servo with L2 & R2 & square
        if msg.buttons[6] == 1: #L2
            vel_0 = -0.5 * (msg.axes[3] - 1.0)
            if math.fabs(vel_0) > self.joy_dead_zone:
                if msg.buttons[0] == 1:
                    self.servo_cmd[0] = -vel_0 * self.vel_rate
                else:
                    self.servo_cmd[1] = vel_0 * self.vel_rate
        if msg.buttons[7] == 1: #R2
            vel_0 = 0.5 * (msg.axes[4] - 1.0)
            if math.fabs(vel_0) > self.joy_dead_zone:
                if msg.buttons[0] == 1:
                    self.servo_cmd[0] = -vel_0 * self.vel_rate
                else:
                    self.servo_cmd[1] = vel_0 * self.vel_rate

        #TODO2: fin control
        fin_angle_ctrl = msg.axes[5] # stick_r vertical
        fin_angle = -fin_angle_ctrl * self.fin_angle_rate + 2048.0
        self.servo_cmd[2] = fin_angle

        # TODO1: Speed Keep
        if msg.buttons[5] == 1: #R2
            self.servo_cmd[0] = self.keep_vel[0]
            self.servo_cmd[1] = self.keep_vel[1]
        else:
            self.keep_vel = [self.servo_cmd[0], self.servo_cmd[1]]

        # pose fish to left or right
        if msg.axes[9] == 1.0:
            self.left_pose()
        if msg.axes[9] == -1.0:
            self.right_pose()

        # sync the two servos to middle position
        if msg.axes[10] != 0.0 or msg.buttons[1]: # front or back
            self.servo_sync()

        #imu feedback
        if msg.buttons[2] ==1:
            self.set_tar_angle()

        if msg.buttons[3] == 1:
            self.imu_feedback(self.tar_angle)

        #for mid-actuation disk, reverse servo 0
        # self.servo_cmd[0] = -self.servo_cmd[0]

        # motor_vel = np.array([int(forward_vel), int(forward_val)], dtype=np.int16)
        msg_0 = ServoControlCmd()
        msg_0.index = [0,1,2]
        msg_0.cmd.append(int(self.servo_cmd[0]))
        msg_0.cmd.append(int(self.servo_cmd[1]))
        msg_0.cmd.append(int(self.servo_cmd[2]))
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
