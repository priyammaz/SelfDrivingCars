import numpy as np
import cv2
import scipy.signal as signal
from numpy import linalg as la
import math

import copy

import rospy
import alvinxy.alvinxy as axy
from sensor_msgs.msg import Image

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

# COLOR_MIN = np.array([5   ,  0.58706456,  0.5 ])
# COLOR_MAX = np.array([40  ,  0.9999999 ,  0.95686275])

COLOR_MIN = np.array([3   , 120 , 80 ])#/255.0
COLOR_MAX = np.array([15  ,  250 ,  250])#/255.0

class FollowWayPoint:
    def __init__(self):
        self.rate = rospy.Rate(25)
        self.pacmod_enable = True
        self.image = None
        self.image_sub = rospy.Subscriber('/zed2/zed_node/stereo/image_rect_color', Image, self.img_callback)
        self.depth_sub = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.depth_callback)
        self.cones = []
        self.cone_locs = []
        
        self.kernel = np.ones((5, 5), np.uint8)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByInertia = False
        params.filterByConvexity = False
        params.minThreshold = 200
        params.maxThreshold = 255
        params.blobColor = 255
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.look_ahead = 4
        self.wheelbase  = 1.75 # meters
        self.offset     = 0.46 # meters

        self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)
        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.speed      = 0.0

        self.olat       = 40.0928563
        self.olon       = -88.2359994
        self.init_lat   = 0.0
        self.init_lon   = 0.0

        # read waypoints into the system 
        self.goal       = 0            
        #self.read_waypoints() 

        self.desired_speed = 1.5  # m/s, reference speed
        self.max_accel     = 0.4 # % of acceleration
        self.pid_speed     = PID(1.2, 0.2, 0.6, wg=20)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)

        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = True

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        # print(image[631:633, 564:567])
        # print(np.min(image[631:633, 564:567], axis=(0,1)))
        # print(np.max(image[631:633, 564:567], axis=(0,1)))
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

        self.set_init_loc = False



    def img_callback(self, msg):

        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[:, :1280, :3]

        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # image = image[466:474,508:516]

        # print(np.min(image, axis=(0,1)))
        # print(np.max(image, axis=(0,1)))
        mask = cv2.inRange(image, COLOR_MIN, COLOR_MAX)
        # cv2.imshow('mask', mask)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # mask[:, :550] = 0
        # mask[:, 800:] = 0

        img_filtered = cv2.GaussianBlur(mask, (5, 5), 0)
        _, img_thresholded = cv2.threshold(img_filtered, 200, 255, cv2.THRESH_BINARY)


        img_dilation = cv2.dilate(img_thresholded, self.kernel, iterations=2)

        kps = self.detector.detect(img_dilation)
        print(len(kps))
        if len(kps) > 0:
            kps = sorted(kps, key=lambda x: x.pt[1])[:2]
            self.cones = np.array([[kp.pt[0], kp.pt[1], kp.size] for kp in kps]).astype(np.int16)
        else:
            self.cones = []

    def depth_callback(self, msg):
        self.cone_locs = []
        cones = copy.copy(self.cones)
        if len(cones) > 0:
            cones = cones.astype(np.int16)
     
            depth_image = np.frombuffer(msg.data, dtype=np.float32)
            depth_image = np.nan_to_num(depth_image)
            depth_image[depth_image>20] = 0
            depth_image = np.clip(depth_image, a_min=0, a_max=20)
            depth_image = depth_image.reshape(msg.height, msg.width)
            # print(np.mean(depth_image))

            depths = []
            for cone in cones:
                # print(cone)
                depths.append(np.mean(depth_image[cone[1] - cone[2]:cone[1] + cone[2], 
                                            cone[0] - cone[2]:cone[0] + cone[2]]))

                # print(cone[1] - cone[2], cone[1] + cone[2], cone[0] - cone[2], cone[0] + cone[2])
            # print(cones)
            # print(depths)
            depths = np.array(depths)
            angles = np.arctan2(cones[:, 1], cones[:, 0] - 640)
            # print(angles)
            # print(depths)
            xs = depths / np.tan(angles)
            xs -= np.sign(xs)*1.5

            # print(depths)

            self.cone_locs = np.stack([xs, depths, angles]).T

    def inspva_callback(self, inspva_msg):
        self.lat     = inspva_msg.latitude  # latitude
        self.lon     = inspva_msg.longitude # longitude
        self.heading = inspva_msg.azimuth   # heading in degrees
        if not self.set_init_loc:
            self.init_lat = self.lat
            self.init_lon = self.lon
            self.set_init_loc = True


    def speed_callback(self, msg):
        self.speed = round(msg.data, 3) # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def heading_to_yaw(self, heading_curr):
        # 0   <= heading < 90  --- 90 to 0     (pi/2 to 0)
        # 90  <= heading < 180 --- 0 to -90    (0 to -pi/2)
        # 180 <= heading < 270 --- -90 to -180 (-pi/2 to -pi)
        # 270 <= heading < 360 --- 180 to 90   (pi to pi/2)
        if (heading_curr >= 0 and heading_curr < 90):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 90 and heading_curr < 180):
            yaw_curr = np.radians(90 - heading_curr)
        elif(heading_curr >= 180 and heading_curr < 270):
            yaw_curr = np.radians(90 - heading_curr)
        else:
            yaw_curr = np.radians(450 - heading_curr)
        return yaw_curr


    def front2steer(self, f_angle):

        if(f_angle > 35):
            f_angle = 35

        if (f_angle < -35):
            f_angle = -35

        if (f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)

        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0

        return steer_angle


    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   

    def get_gem_state(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)

        # return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)
        return round(curr_x, 3), round(curr_y, 3), round(self.heading)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def start_pacmod(self):
        while not rospy.is_shutdown():
            # print(self.gem_enable, self.pacmod_enable)
            if not self.gem_enable:
                if self.pacmod_enable:
                    # ---------- enable PACMod ----------

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

                    self.gem_enable = True
            
            curr_x, curr_y, curr_azimuth = self.get_gem_state()
            # print(self.cone_locs)
            if len(self.cone_locs) > 0:

                target_loc_x = self.cone_locs[0, 0]
                target_loc_y = self.cone_locs[0, 1]
                target_loc_angle = np.arctan2(target_loc_y, target_loc_x)# self.cone_locs[0, 2]
                print(curr_x, curr_y, target_loc_x, target_loc_y)
                print("current azimuth(degree", curr_azimuth*180/np.pi)
                print("target location angle (degree)", target_loc_angle*180/np.pi)
                target_loc_angle_azimuth = np.pi/2 - target_loc_angle
                # local cone yaw -> local cone azimuth
                # ??????
                # if target_loc_angle < 0:
                #     target_loc_angle = 90 - target_loc_angle
                # if target_loc_angle < 90:
                #     target_loc_angle = 90 - target_loc_angle
                # else:
                #     target_loc_angle = -(target_loc_angle - 90)
                

                cone_azimuth = target_loc_angle_azimuth + curr_azimuth
                car_azimuth_rad = (-curr_azimuth)
                rot_mat = np.array([[np.cos(car_azimuth_rad), -np.sin(car_azimuth_rad)],
                                    [np.sin(car_azimuth_rad), np.cos(car_azimuth_rad)]])

                # print(curr_x, curr_y, curr_azimuth)
                rotated = rot_mat @ np.array([[target_loc_x], [target_loc_y]])
                # print(rotated)
                updated = (rotated + np.array([[curr_x], [curr_y]])).flatten()
                world_cone_x = updated[0]
                world_cone_y = updated[1]
                
                # target_loc_x = curr_x + np.cos(target_loc_relative_angle * np.pi / 180.0) * target_loc_x
                # target_loc_y = curr_y + np.sin(target_loc_relative_angle * np.pi / 180.0) * target_loc_y
                if len(self.cone_locs) > 0:
                    
                    L = self.dist((world_cone_x, world_cone_y), (curr_x, curr_y))
                    # L = np.sqrt(np.sum(self.cone_locs[0, :2] ** 2))
                    alpha = target_loc_angle_azimuth
                    # adding may have made it outside the target range
                    # if -360 <= alpha and alpha < -180:
                    #     alpha = 360 + alpha
                    # elif -180 <= alpha and alpha < 0:
                    #     continue
                    # elif 0 <= alpha and alpha < 180:
                    #     continue
                    # else:
                    #     alpha -= 360
                    
                    k       = 0.41 
                    angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L) 
                    angle   = angle_i*2


                    f_delta = round(np.clip(angle, -0.61, 0.61), 3)

                    f_delta_deg = np.degrees(f_delta)

                    # steering_angle in degrees
                    steering_angle = self.front2steer(f_delta_deg)
                    steering_angle = -steering_angle
                    

                    if(self.gem_enable == True):
                        # print("Current index: " + str(self.goal))
                        # print("Forward velocity: " + str(self.speed))
                        ct_error = round(np.sin(alpha) * L, 3)
                        # print("Crosstrack Error: " + str(ct_error))
                        # print("Front steering angle: " + str(np.degrees(f_delta)) + " degrees")
                        print("Steering wheel angle: " + str(steering_angle) + " degrees" )
                        print("alpha: (degrees)", alpha/np.pi*180)
                        print("\n")

                # if (self.goal >= 625 and self.goal <= 940):
                #     self.desired_speed = 1.5
                # else:
                #     self.desired_speed = 0.7

                    current_time = rospy.get_time()
                    filt_vel     = self.speed_filter.get_data(self.speed)
                    output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)

                    if output_accel > self.max_accel:
                        output_accel = self.max_accel

                    if output_accel < 0.3:
                        output_accel = 0.3

                    if (f_delta_deg <= 30 and f_delta_deg >= -30):
                        self.turn_cmd.ui16_cmd = 1
                    elif(f_delta_deg > 30):
                        self.turn_cmd.ui16_cmd = 2 # turn left
                    else:
                        self.turn_cmd.ui16_cmd = 0 # turn right


                    print(self.accel_cmd)
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = np.radians(steering_angle)
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)

            self.rate.sleep()



class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0

    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted
        

def pacmod_run():

    rospy.init_node('pacmod_control_node', anonymous=True)
    pacmod =  FollowWayPoint()

    try:
        pacmod.start_pacmod()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == '__main__':
    pacmod_run()



