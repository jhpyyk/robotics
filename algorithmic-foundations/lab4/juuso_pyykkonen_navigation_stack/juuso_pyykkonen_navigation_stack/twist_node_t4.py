#! /usr/bin/env python

from cmath import polar
from math import floor
from re import X
from turtle import update
from urllib import response
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from interfaces.srv import NextWaypoint
import numpy as np
import time
import matplotlib.pyplot as plt
import traceback
from gradient_t4 import gradient


class Twister(Node):
    def __init__(self):
        super().__init__('twister')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.odom = Odometry()

        self.scan = LaserScan()

        self.subscription = self.create_subscription(
            String,
            '/grid_map',
            self.map_callback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)


        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path_client = self.create_client(NextWaypoint, 'next_waypoint')
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for next waypoint service')
        self.req = NextWaypoint.Request()

        self.waiting_for_waypoint = True

        self.twist_msg = Twist()

        self.map_path =''

        self.next_waypoint = [0.0, 0.0]
        self.first_scan = True

        self.angular_speed = 0.8
        self.linear_speed = 0.4

        self.resolution = 15
        self.grid_size =  10*int(np.ceil(self.resolution * 3.5))

        self.trajectory_record = []

        self.state = 'stopped'
        self.yaw = 0

        self.twist_commands_linear = []
        self.twist_commands_angular = []
        self.time_stamps_linear = []
        self.time_stamps_angular = []

        self.time_start = time.time()


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


    def is_at_waypoint(self):
        diff_x = abs(self.odom.pose.pose.position.x - self.next_waypoint[0])
        diff_y = abs(self.odom.pose.pose.position.y - self.next_waypoint[1])
        if (diff_x < 0.4 and diff_y < 0.4):
            return True
        else:
            return False

    def angle_between_points(self, point_a, point_b, point_c):
        
        vector_ba = [point_a[0] - point_b[0], point_a[1] - point_b[1]]
        vector_bc = [point_c[0] - point_b[0], point_c[1] - point_b[1]]

        cosine_angle = np.dot(vector_ba, vector_bc) / (np.linalg.norm(vector_ba) * np.linalg.norm(vector_bc))
        angle = np.arccos(cosine_angle)
        return angle

    def calc_angle_waypoint(self, grad, yaw):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        point_a = [x + np.cos(yaw), y +  np.sin(yaw)]
        point_b = [x, y]
        point_c = [x + grad[0], y + grad[1]]

        angle = self.angle_between_points(point_a, point_b, point_c)

        return angle

    def adjust_trajectory(self, grad):
        if (self.state != 'adjusting'):
            self.state = 'adjusting'
            angle_plus = self.calc_angle_waypoint(grad, self.yaw + np.pi/60)
            angle_minus = self.calc_angle_waypoint(grad, self.yaw - np.pi/60)
            if (angle_minus < angle_plus):
                self.twist_msg.angular.z = -self.angular_speed
            else:
                self.twist_msg.angular.z = self.angular_speed
            self.publisher_.publish(self.twist_msg)

            # recording commands
            self.twist_commands_angular.append(self.twist_msg.angular.z)
            self.time_stamps_angular.append(time.time()- self.time_start)
            self.twist_commands_linear.append(self.twist_msg.linear.x)
            self.time_stamps_linear.append(time.time()- self.time_start)
        


    def go_straight(self):
        if (self.state != 'going straight'):
            self.state = 'going straight'
            self.last_angular = self.twist_msg.angular.z
            self.twist_msg.angular.z = 0.0
            self.twist_msg.linear.x = self.linear_speed
            self.publisher_.publish(self.twist_msg)

            # recording commands
            self.twist_commands_angular.append(self.twist_msg.angular.z)
            self.time_stamps_angular.append(time.time() - self.time_start)
            self.twist_commands_linear.append(self.twist_msg.linear.x)
            self.time_stamps_linear.append(time.time() - self.time_start)

    def stop(self):
        if (self.state != 'stopped'):
            self.state = 'stopped'
            self.twist_msg.angular.z = 0.0
            self.twist_msg.linear.x = 0.0
            self.publisher_.publish(self.twist_msg)

            # recording commands
            self.twist_commands_angular.append(self.twist_msg.angular.z)
            self.time_stamps_angular.append(time.time() - self.time_start)
            self.twist_commands_linear.append(self.twist_msg.linear.x)
            self.time_stamps_linear.append(time.time() - self.time_start)

    def update_trajectory_record(self):
        point = [0,0]
        point[0] = int(np.floor(self.grid_size/2) - self.resolution * self.odom.pose.pose.position.x)
        point[1] = int(np.floor(self.grid_size/2) - self.resolution * self.odom.pose.pose.position.y)
        self.trajectory_record.append(point)
            
    def map_plot(self):
        self.state = 'plotting'
        map =  np.genfromtxt(self.map_path, delimiter=',', ndmin=2)
        for point in self.trajectory_record:
            map[point[0],point[1]] = 0.5
        plt.matshow(map, cmap='bone')
        fig = plt.figure()
        ax1 = fig.add_subplot(211)
        ax1.set_title('Linear speed')
        ax1.set_xlabel('seconds')
        ax1.plot(self.time_stamps_linear, self.twist_commands_linear)
        ax2 = fig.add_subplot(212)
        ax2.set_title('Angular speed')
        ax2.set_xlabel('seconds')
        ax2.plot(self.time_stamps_angular, self.twist_commands_angular)
        fig.tight_layout()
        plt.show()

    def request_waypoint(self):
        print('requesting next waypoint')
        self.req.x = self.odom.pose.pose.position.x
        self.req.y = self.odom.pose.pose.position.y
        self.future = self.path_client.call_async(self.req)
        

    def odom_callback(self, msg):
        self.odom = msg
        qtrn = self.qtrn_msg_to_array(self.odom.pose.pose.orientation)
        self.yaw = self.qtrn_yaw(qtrn)
        if ( self.first_scan):
            self.request_waypoint()
            self.first_scan = False

    def map_callback(self, msg):
        self.map_path = msg.data

    def scan_callback(self, msg):
        self.scan = msg

        pos = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]

        grad = gradient(self.scan, pos, self.next_waypoint, self.yaw)
        grad[0] = -grad[0]
        grad[1] = -grad[1]

        angle = self.calc_angle_waypoint(grad, self.yaw)

        if (self.waiting_for_waypoint):
            if (self.future.done()):
                try:
                    response = self.future.result()
                    if (response.all_visited):
                        self.stop()
                        print('all visited')
                        self.map_plot()
                    self.next_waypoint = [response.x, response.y]
                    print(self.next_waypoint)
                    self.waiting_for_waypoint = False
                except Exception as e:
                    print('service call failed')
            else:
                return

        if (not self.waiting_for_waypoint):
            if (self.is_at_waypoint()):
                self.stop()
                self.request_waypoint()
                self.waiting_for_waypoint = True            

            elif (angle > np.pi/40):
                self.adjust_trajectory(grad)
            else:
                self.go_straight()
        


def main(args=None):
    rclpy.init(args=args)
    twister = Twister()
    rclpy.spin(twister)
    twister.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()