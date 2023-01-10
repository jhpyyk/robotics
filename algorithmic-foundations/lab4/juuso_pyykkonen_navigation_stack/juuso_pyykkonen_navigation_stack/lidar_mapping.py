#! /usr/bin/env python

from cmath import polar
from math import floor
from re import X
from turtle import update
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import time
import matplotlib.pyplot as plt
import traceback

class OdomAndScan(Node):
    def __init__(self):
        super().__init__('odom_and_scan')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.scan = None
        self.odom = None


        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_policy)

        self.publisher_ = self.create_publisher(String, '/grid_map', 10)

        self.map_path = '../map/map.csv'

        self.resolution = 15

        self.grid_size =  10*int(np.ceil(self.resolution * 3.5))

        self.position = [int(np.floor(self.grid_size/2)), int(np.floor(self.grid_size/2))]

        self.map = np.full((self.grid_size, self.grid_size), -1)
        print("init")


        #self.publisher_ = self.create_publisher(LaserScan, '/scan_line_detector', 10)


    def scan_callback(self,msg):

        self.scan = msg
    def odom_callback(self, msg):

        self.odom = msg


    def update_position(self):
        self.position[0] = int(np.floor(self.grid_size/2) - self.resolution * self.odom.pose.pose.position.x)
        self.position[1] = int(np.floor(self.grid_size/2) - self.resolution * self.odom.pose.pose.position.y)
        print(self.position)


    def grid_map(self):

        self.update_position()
        
        # maps the robot trace
        #self.map[self.position[0]][self.position[1]] = 1

        scaled_ranges =  self.scale_ranges(self.scan.ranges, self.resolution)


        orientation_qtrn = self.qtrn_msg_to_array(self.odom.pose.pose.orientation)

        position_msg = self.odom.pose.pose.position
        print(position_msg)

        yaw = self.qtrn_yaw(orientation_qtrn)
        print('Yaw')
        print(yaw)

        cartesian_points = self.polar_to_cartesian_coordinate(scaled_ranges, self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment, yaw)

        points = self.integer_points(cartesian_points)


        
        bres = self.bres_points(points)
        

        for point in bres:
            x = -point[0]
            y = -point[1]
            if self.map[self.position[0] + x][self.position[1] + y] == -1:
                self.map[self.position[0] + x][self.position[1] + y] = 0

        for point in points:
            x = -point[0]
            y = -point[1]
            xy_length = np.sqrt(x**2 + y**2)/self.resolution
            if xy_length < 3.3:
                if self.map[self.position[0] + x][self.position[1] + y] == -1:
                    self.map[self.position[0] + x][self.position[1] + y] = 1
            else:
                if self.map[self.position[0] + x][self.position[1] + y] == -1:
                    self.map[self.position[0] + x][self.position[1] + y] = 0


    def save_map(self):
        np.savetxt(self.map_path, self.map, delimiter=',')
        pub = String()
        pub.data = self.map_path
        self.publisher_.publish(pub)




        

    def qtrn_msg_to_array(self, qtrn_msg):
        qtrn = []
        qtrn.append(qtrn_msg.x)
        qtrn.append(qtrn_msg.y)
        qtrn.append(qtrn_msg.z)
        qtrn.append(qtrn_msg.w)
        return qtrn


    def qtrn_yaw(self, qtrn):
        w = qtrn[3]
        x = qtrn[0]
        y = qtrn[1]
        z = qtrn[2]
        yaw = np.arctan2(2*(w * z + x * y), w * w + x * x - y * y - z * z)
        if yaw < 0:
            yaw = 2*np.pi + yaw
        return yaw
        
    def scale_ranges(self, ranges, scale):
        scaled_ranges = []
        for range in ranges:
            range = self.check_range(range)
            scaled_ranges.append(range * scale)
        return scaled_ranges

    def integer_points(self, points):
        points_int = []
        for point in points:
            points_int.append([int(point[0]), int(point[1])])
        return points_int

    def bres_points(self, points):
        bres = []
        for point in points:
            x2 = point[0]
            y2 = point[1]
            bres.extend(self.bresenham_points([0, 0], [x2, y2]))
        return bres




    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, angle_increment, angle_offset):

 
        angle = angle_min + angle_offset           # start angle  
        tf_points = []         
        for range in ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)

            # current angle is last angle add angle_increment
            angle += angle_increment     
            tf_points.append([x,y])

        # transform the transformed points to numpy array
        points_np = np.array(tf_points)
        return points_np

    def check_range(self, point):
        if abs(point) > self.scan.range_max:
            point = self.scan.range_max
        return point


    def bresenham_points (self, p0, p1):
        
        point_list = []  # We will fill this list with all points in between p0 and p1
        
        x0, y0 = p0[0], p0[1]
        x1, y1 = p1[0], p1[1]

        dx = abs(x1-x0)
        dy = abs(y1-y0)
        if x0 < x1:
            sx = 1
        else:
            sx = -1

        if y0 < y1:
            sy = 1
        else:
            sy = -1
            
        err = dx-dy

        while True:
            #print("{}, {}".format(x0, y0))
            point_list.append([x0, y0])
            if x0 == x1 and y0 == y1:
                break # This means we have finished, so we break the loop
                
            e2 = 2*err
            if e2 > -dy:
                # overshot in the y direction
                err = err - dy
                x0 = x0 + sx
            if e2 < dx:
                # overshot in the x direction
                err = err + dx
                y0 = y0 + sy
        
        point_list = point_list[1:-1]
        return point_list



 
def main(args=None):
    rclpy.init(args=args)
    odom_and_scan = OdomAndScan()
    
    # Init odom_and_scan
    #odom_and_scan.pf.init_filter()

    time.sleep(1)
    rclpy_check_rate = 10
    
    rclpy_check_rate = odom_and_scan.create_rate(10, odom_and_scan.get_clock())

    odom_and_scan.get_logger().info("Starting particle odom_and_scan...")
    try:
        try:
            while rclpy.ok() :
                scan_timer = odom_and_scan.create_timer(0.5, odom_and_scan.grid_map)
                map_timer = odom_and_scan.create_timer(1, odom_and_scan.save_map)
                rclpy.spin(odom_and_scan)             
                pass
            

        except KeyboardInterrupt :
            odom_and_scan.get_logger().error('Keyboard Interrupt detected! Trying to stop odom_and_scan node!')
    except Exception as e:
        odom_and_scan.destroy_node()
        odom_and_scan.get_logger().info("UWB particle odom_and_scan failed %r."%(e,))
        print(traceback.format_exc())
    finally:
        #plt.matshow(odom_and_scan.map, cmap='bone')
        #plt.show()
        rclpy.shutdown()
        odom_and_scan.destroy_node() 


if __name__ == '__main__':
    main()