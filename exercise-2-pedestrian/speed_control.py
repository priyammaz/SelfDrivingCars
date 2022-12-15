#!/usr/bin/env python3

#==============================================================================
# File name          : gem_pacmod_control.py                                                                  
# Description        : pacmod interface                                                             
# Author             : Hang Cui
# Email              : hangcui3@illinois.edu                                                                     
# Date created       : 08/08/2022                                                                 
# Date last modified : 08/18/2022                                                          
# Version            : 1.0                                                                    
# Usage              : rosrun gem_pacmod_control gem_pacmod_control.py                                                                   
# Python version     : 3.8                                                             
#==============================================================================

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
from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt

import matplotlib.pyplot as plt

class PACMod(object):
    
    def __init__(self):

        self.rate = rospy.Rate(25)

        self.stanley_sub = rospy.Subscriber('/gem/stanley_gnss_cmd', AckermannDrive, self.stanley_gnss_callback)

        self.ackermann_msg_gnss                         = AckermannDrive()
        self.ackermann_msg_gnss.steering_angle_velocity = 0.0
        self.ackermann_msg_gnss.acceleration            = 0.0
        self.ackermann_msg_gnss.jerk                    = 0.0
        self.ackermann_msg_gnss.speed                   = 0.0 
        self.ackermann_msg_gnss.steering_angle          = 0.0

        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = True

        # GEM vehicle enable
        self.enable_sub = rospy.Subscriber('/pacmod/as_rx/enable', Bool, self.pacmod_enable_callback)
        # self.enable_cmd = Bool()
        # self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 2.0 # radians/second

        self.lidarSub = rospy.Subscriber("/lidar1/velodyne_points", PointCloud2, self.lidar_callback)
        self.speedSub = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.velocity_callback)
        self.yoloSub = rospy.Subscriber("/yolo_detect", Float64MultiArray, self.yolo_callback)
        self.boxes = np.zeros((4, 0))
        self.detect_pedestrian = False 
        # subscribe to parsed_tx/vehicle_speed_rpt

        # for pid control
        # --------- Tune PID here.
        self.kp = 0.06#0.01
        self.ki = 0.0001#0.0001
        self.kd = 0.01
        # ---------
        self.speed = 0.0
        self.accum_error = 0.0
        self.pre_error = 0.0

        self.buffer = []
    def yolo_callback(self, msg):
        flatten_array = np.array(msg.data)
        n = len(flatten_array)
        self.boxes = flatten_array.reshape(int(n/4), 4).T
    # Get outputs of Stanley controller based on GNSS
    def stanley_gnss_callback(self, msg):
        self.ackermann_msg_gnss.acceleration = round(msg.acceleration ,2)
        self.ackermann_msg_gnss.steering_angle = round(msg.steering_angle ,2)

    def lidar_callback(self, msg):
        # print(msg.data)
        pass

    def velocity_callback(self, msg):
        self.speed = msg.data

    # PACMod enable callback function
    def pacmod_enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def pid_control(self, target, measure):
        error = target - measure
        self.accum_error += error
        if(abs(error) < 0.01):
            # if(self.accum_error > 0):
            #     self.accum_error = 50
            # else:
            #     self.accum_error = -50
            self.accum_error = 0.0
        p_error = self.kp * error
        i_error = self.ki * self.accum_error
        d_error = self.kd * (error - self.pre_error)
        self.pre_error = error
        return p_error + i_error + d_error

    def speedControl(self, target):
        measure = self.speed
        self.buffer.append(measure)
        if(len(self.buffer) > 10):
            self.buffer.pop(0)
        measure_mean = np.mean(self.buffer)
        print("vehicle avg speed: ",measure_mean)
        output = self.pid_control(target, measure_mean)
        print(f"output PID {output}")
        if(output >= 0):
            # accel
            output += 0.30
            if(output > 1):
                output = 1.0
            self.accel_cmd.f64_cmd = abs(output)
            self.brake_cmd.f64_cmd = 0.0
        else:
            # brake
            output -= 0.3
            if(output < -1):
                output = -1.0
            self.accel_cmd.f64_cmd = 0.0
            
            self.brake_cmd.f64_cmd = abs(output)

        # self.accel_cmd.f64_cmd = 0.5
        print(f"accel cmd: {round(self.accel_cmd.f64_cmd, 3)}, break: {round(self.brake_cmd.f64_cmd, 3)}")
        self.accel_pub.publish(self.accel_cmd)
        self.brake_pub.publish(self.brake_cmd)

    def sensor_init(self):
        # ---------- Enable PACMod ----------
        # enable forward gear
        self.gear_cmd.ui16_cmd = 3

        # enable brake
        self.brake_cmd.enable  = True
        self.brake_cmd.clear   = False
        self.brake_cmd.ignore  = False
        self.brake_cmd.f64_cmd = 0.0

        # enable gas 
        self.accel_cmd.enable  = True
        self.accel_cmd.clear   = False
        self.accel_cmd.ignore  = False
        self.accel_cmd.f64_cmd = 0.0

        self.gear_pub.publish(self.gear_cmd)
        print("Foward Engaged!")

        self.turn_pub.publish(self.turn_cmd)
        print("Turn Signal Ready!")
        
        self.brake_pub.publish(self.brake_cmd)
        print("Brake Engaged!")

        self.accel_pub.publish(self.accel_cmd)
        print("Gas Engaged!")
    # Start PACMod interface
    def start_pacmod(self):
        while not rospy.is_shutdown():
            # print(f"loop")
            if(self.pacmod_enable == True):
                if (self.gem_enable == False):
                    print(f"hi")
                    self.sensor_init()
                    self.gem_enable = True
                else: 
                    # Regularly where we run.
                    if(not self.detect_pedestrian):
                        targetSpeed = 1.2#1.5
                    else:
                        targetSpeed = 0.0
                    print(self.boxes)
                    if(self.boxes.shape[1] > 1):
                        # there's people
                        depth = self.boxes[2][1] # first person
                        if(depth < 90):
                            self.detect_pedestrian = True
                            print("Close priyam!!!!!!!!!")
                        else:
                            self.detect_pedestrian = False
                            print("No close priyam")
                    else:
                        self.detect_pedestrian = False
                        print("No priyam")
                    print(f"target speed: {targetSpeed}")
                    self.speedControl(targetSpeed)
            self.rate.sleep()


def pacmod_run():

    rospy.init_node('pacmod_control_node', anonymous=True)
    pacmod =  PACMod()

    try:
        pacmod.start_pacmod()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == '__main__':
    pacmod_run()


