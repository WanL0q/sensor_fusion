#!/usr/bin/python
import rospy
import utm
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix,Imu
from geometry_msgs.msg import Point 
# from cargobot_msgs.msg import Route
import matplotlib.pyplot as plt
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Vehicle parameters
LENGTH = 0.8  # [m]
WIDTH = 0.5  # [m]
BACKTOWHEEL = LENGTH/2  # [m]
WHEEL_LEN = 0.08  # [m]
WHEEL_WIDTH = 0.05  # [m]
TREAD = WIDTH/2  # [m]
WB = 0.46  # [m]
HEAD = 0.05

distance_gps2ori = 0.3 # [m]


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):

    outline = np.matrix([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
    head = np.matrix([[outline[0,1] - HEAD, outline[0,1], outline[0,1] ,  outline[0,1] -HEAD,outline[0,1] - HEAD ],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.matrix([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                          [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.matrix([[math.cos(steer), math.sin(steer)],
                      [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T * Rot2).T
    fl_wheel = (fl_wheel.T * Rot2).T
    fr_wheel[0, :] += WB/2
    fl_wheel[0, :] += WB/2
    rr_wheel[0, :] += -WB/2
    rl_wheel[0, :] += -WB/2

    fr_wheel = (fr_wheel.T * Rot1).T
    fl_wheel = (fl_wheel.T * Rot1).T

    outline = (outline.T * Rot1).T
    rr_wheel = (rr_wheel.T * Rot1).T
    rl_wheel = (rl_wheel.T * Rot1).T
    head = (head.T * Rot1).T

    outline[0, :] += x
    outline[1, :] += y
    head[0, :] += x
    head[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(head[0, :]).flatten(),
             np.array(head[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")

class PlotGPS:
    def __init__(self):
        rospy.init_node('plot_GPS_node', anonymous = True)
        self.fusion_sub = rospy.Subscriber('ekf4/gps/filtered',NavSatFix,self.fusion_callback)
        self.gps_sub = rospy.Subscriber('/ublox/fix',NavSatFix,self.gps_callback)
        self.imu_sub = rospy.Subscriber('/imu/data',Imu,self.imuCallback)
        self.utm_origin_sub = rospy.Subscriber('ekf4/utm/origin',Point,self.utm_ori_callback)
        self.yaw_imu = 0.0
        self.utm_ori_x = 0
        self.utm_ori_y = 0
        self.list_gps_x = []
        self.list_gps_y = []
        self.list_yaw =[]
        self.list_fusion_x = []
        self.list_fusion_y = []
        self.fusion_flag = False
        self.gps_flag = False
        self.imu_flag = False
        self.utm_ori_flag = False
        self.rate = rospy.Rate(10) # 10Hz
        self.flag_plot = False
        self.list_point_x = []
        self.list_point_y = []
        # self.route_msg = Route()
        self.route_flag = False

    def imuCallback(self,msg):
        orientation_q = msg.orientation
        list_q = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        roll, pitch, self.yaw_imu = euler_from_quaternion(list_q)
        self.imu_flag = True



    def route_callback(self,msg):
        self.route_msg = msg
        self.route_flag = True
    
    def fusion_callback(self,msg):
        # self.fusion_flag = True
        # orientation_q = msg.pose.pose.orientation
        # list_q = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        # roll, pitch, self.yaw_imu = euler_from_quaternion(list_q)
        # utm_x = msg.pose.pose.position.x
        # utm_y = msg.pose.pose.position.y
        long = msg.longitude
        lat = msg.latitude
        easting, northing, ZONE_NUMBER, ZONE_LETTER = utm.from_latlon(lat, long)
        self.list_fusion_x.append(easting)
        self.list_fusion_y.append(northing)
    
       
    def gps_callback(self,msg):
        long = msg.longitude
        lat = msg.latitude
        EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER = utm.from_latlon(lat, long)
        if msg.position_covariance[0] > 50  :
            self.flag_plot = False
        else :
            self.flag_plot = True
       
        self.gps_flag = True
        # if (self.imu_flag):
        self.list_gps_x.append(EASTING )
        self.list_gps_y.append(NORTHING )
        # self.list_yaw.append(self.yaw_imu)
    

    # def gps_callback(self,msg):
    #     long = msg.longitude
    #     lat = msg.latitude
    #     EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER = utm.from_latlon(lat, long)
    #     if msg.position_covariance[0] > 50  :
    #         self.flag_plot = False
    #     else :
    #         self.flag_plot = True    
    #     self.gps_flag = True
    #     if self.utm_ori_flag :
    #         self.list_gps_x.append(EASTING - self.utm_ori_x)
    #         self.list_gps_y.append(NORTHING - self.utm_ori_y)  
    
    def utm_ori_callback(self,msg):
        
        self.utm_ori_x = msg.x
        self.utm_ori_y = msg.y
        self.utm_ori_flag = True
    
    def plot_fusion(self):
        timer = rospy.get_time()
        # while  not self.gps_flag and not self.route_flag and not rospy.is_shutdown():
        #     if rospy.get_time() - timer > 1.0 :
        #         timer = rospy.get_time()
        #         rospy.loginfo("Waiting for callback ... ")
        #     else:
        #         self.rate.sleep()
        
        while not rospy.is_shutdown():
            # if self.route_flag:
            #     self.route_flag = False
            #     self.list_point_x = []
            #     self.list_point_y = []
            #     # self.list_gps_x = []
            #     # self.list_gps_y = []
            #     for point in self.route_msg.list_point :
            #         self.list_point_x.append(point.x)
            #         self.list_point_y.append(point.y)
            #     print(self.list_point_x)
            if self.gps_flag :
                self.gps_flag = False
                plt.figure("Plot")
                plt.cla()
                plt.title("Plot GPS")
                plt.axis("equal")
                plt.grid(True)
                l = len(self.list_gps_x)
                if l < 100 :
                    # plt.plot(self.list_point_x,self.list_point_y,'--r',label='route')
                    plt.quiver(self.list_gps_x,self.list_gps_y, np.cos(self.list_yaw), np.sin(self.list_yaw) ,width = 0.001, linewidth = 0.5, color='r')
                    # plt.plot(self.list_gps_x,self.list_gps_y,'-b',label="gps")
                    # plot_car(self.list_gps_x[-1],self.list_gps_y[-1],self.yaw_imu) 
                else:
                    plt.quiver(self.list_gps_x[l-100:l],self.list_gps_y[l-100:l], np.cos(self.list_yaw[l-100:l]), np.sin(self.list_yaw[l-100:l]) ,width = 0.001, linewidth = 0.5, color='r')
                    # plt.plot(self.list_gps_x[l-100:l],self.list_gps_y[l-100:l],'-b',label="gps")
                    # plot_car(self.list_gps_x[-1],self.list_gps_y[-1],self.yaw_imu)
                
                    
                plt.legend()
                plt.pause(0.1)
                # plt.show()
                # self.rate.sleep()
    def plot_gps(self):
        timer = rospy.get_time()
        while not self.gps_flag and not self.fusion_flag and not rospy.is_shutdown():
            if rospy.get_time() - timer > 1.0 :
                timer = rospy.get_time()
                rospy.loginfo("Waiting for callback ... ")
            else:
                self.rate.sleep()
        
        while not rospy.is_shutdown():
            plt.figure("Plot")
            plt.cla()
            plt.title("Plot GPS")
            plt.axis("equal")
            plt.grid(True)
            # if self.flag_plot :
            plt.plot(self.list_gps_x,self.list_gps_y,'-b',label="gps")
            plt.plot(self.list_fusion_x,self.list_fusion_y,'-r',label='fusion')
            # if len(self.list_fusion_x) > 0:
            #     plot_car(self.list_gps_x[-1],self.list_gps_x[-1],self.yaw_imu)
            
            plt.legend()
            plt.pause(0.1)
            # plt.show()
            # self.rate.sleep()
    
    def main(self):
        # self.plot_fusion()
        self.plot_gps()
       


if __name__ == "__main__" :
    try:
        plot = PlotGPS()
        plot.main()
    except KeyboardInterrupt :
        print("\n Exit")

        
                


    


