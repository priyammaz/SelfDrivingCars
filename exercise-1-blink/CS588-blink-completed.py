from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import rospy
import alvinxy.alvinxy as axy 
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String, Bool, Float32, Float64, Header
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt

class Blink:
    def __init__(self):
        self.rate = rospy.Rate(0.5)
        self.pacmod_enable = True
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 2 # None
        self.num_blinks = -1

      # PACMod enable callback function
    def blinker_callback(self, msg):
        self.pacmod_enable = msg.data

    def start_pacmod(self):
        while not rospy.is_shutdown():
            if(self.pacmod_enable == True):
                # if time.time() - self.start_time < 2 or (time.time() - self.start_time >= 4 and time.time() - self.start_time < 6):
                #     self.turn_cmd.ui16_cmd = 2
                # elif time.time() - self.start_time >= 2 and time.time() - self.start_time < 4:
                #     self.turn_cmd.ui16_cmd = 0
                # else:
                #     self.turn_cmd.ui16_cmd = 1
                # self.turn_pub.publish(self.turn_cmd)
                # self.rate.sleep()


                if self.num_blinks == 0 or self.num_blinks == 2:
                    self.turn_cmd.ui16_cmd = 2
                    self.num_blinks += 1
                elif self.num_blinks == 1:
                    self.turn_cmd.ui16_cmd = 0
                    self.num_blinks += 1
                else:
                    self.turn_cmd.ui16_cmd = 1
                    self.num_blinks += 1

                self.turn_pub.publish(self.turn_cmd)
                self.rate.sleep()
                
                    # self.rate.sleep()
                # for i in range(3):
                #     self.turn_cmd.ui16_cmd = 0
                #     self.turn_pub.publish(self.turn_cmd)


                    # self.rate.sleep()
                # self.turn_cmd.ui16_cmd = 1
                # self.turn_pub.publish(self.turn_cmd)


def pacmod_run():

    rospy.init_node('pacmod_control_node', anonymous=True)
    pacmod = Blink()


    try:
        pacmod.start_pacmod()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pacmod_run()
