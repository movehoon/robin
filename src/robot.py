#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

from threading import Thread, Timer

import time

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor


class RobinRobot(Thread):

    def pcaInit(self):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c, address=0x60)
        self.pca.frequency = 100

        self.pca.channels[8].duty_cycle = 0xFFFF
        self.motor1 = motor.DCMotor(self.pca.channels[10], self.pca.channels[9])
        self.pca.channels[13].duty_cycle = 0xFFFF
        self.motor2 = motor.DCMotor(self.pca.channels[11], self.pca.channels[12])
        self.pca.channels[2].duty_cycle = 0xFFFF
        self.motor3 = motor.DCMotor(self.pca.channels[4], self.pca.channels[3])
        self.pca.channels[7].duty_cycle = 0xFFFF
        self.motor4 = motor.DCMotor(self.pca.channels[5], self.pca.channels[6])
        pass

    def motor(self, fl, fr, rl, rr):
        rospy.loginfo('motor %f %f %f %f', fl, fr, rl, rr)
        self.motor1.throttle = fr
        self.motor2.throttle = rr
        self.motor3.throttle = rl
        self.motor4.throttle = fl
        pass

    def mecanum(self, x_vel, y_vel, a_vel):
        rospy.loginfo('mecanum linear:' + str(x_vel) + ', angular: ' + str(a_vel))
        fl = a_vel + y_vel - x_vel
        fr = a_vel + y_vel + x_vel
        rl = a_vel - y_vel - x_vel
        rr = a_vel - y_vel + x_vel
        self.motor(fl, fr, rl, rr)
        pass

    def mobility(self, twist):
        rospy.loginfo('mobility' + str(twist));
        self.mecanum(twist.linear.x, twist.linear.y, twist.angular.z)
        pass

    def __init__(self):
        self.pcaInit()
        self.mecanum(0, 0, 0)

        self.subCmdVel = rospy.Subscriber('cmd_vel', Twist, self.mobility, queue_size=1)

        Thread.__init__(self)

        rospy.loginfo('Robin robot initialized')

    def run(self):
        rospy.loginfo('thread run')
        self._run = True
        while self._run:
            rospy.sleep(0.1)

        self.mecanum(0, 0, 0)
        rospy.loginfo('thread stopped')

    def stop(self):
        rospy.loginfo('thread stop')
        self._run = False

if __name__ == '__main__':
    try :
        rospy.init_node('robin_robot')
        server = RobinRobot()
        server.start()
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
        server.stop()
    except rospy.ROSInterruptException:
        server.stop()
        pass


