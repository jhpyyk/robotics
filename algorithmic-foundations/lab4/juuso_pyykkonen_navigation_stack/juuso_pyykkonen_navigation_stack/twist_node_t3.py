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
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import numpy as np
import time
import matplotlib.pyplot as plt
import traceback


class Twister(Node):
    def __init__(self):
        super().__init__('twister')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.odom = Odometry()

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
            Float64MultiArray,
            '/waypoints',
            self.waypoints_callback,
            qos_profile=qos_policy)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path_client = self.create_client(Trigger, 'trigger')
        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for path service')
        self.req = Trigger.Request()

        self.waiting_for_path = False

        self.twist_msg = Twist()

        self.map_path =''

        self.path = []
        self.path_index = 0

        self.angular_speed = 0.4
        self.linear_speed = 0.5

        self.resolution = 10
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


    def is_at_waypoint(self, waypoint):
        diff_x = abs(self.odom.pose.pose.position.x - waypoint[0])
        diff_y = abs(self.odom.pose.pose.position.y - waypoint[1])
        if (diff_x < 0.2 and diff_y < 0.2):
            return True
        else:
            return False

    def angle_between_points(self, point_a, point_b, point_c):
        
        vector_ba = [point_a[0] - point_b[0], point_a[1] - point_b[1]]
        vector_bc = [point_c[0] - point_b[0], point_c[1] - point_b[1]]

        cosine_angle = np.dot(vector_ba, vector_bc) / (np.linalg.norm(vector_ba) * np.linalg.norm(vector_bc))
        angle = np.arccos(cosine_angle)
        return angle

    def calc_angle_waypoint(self, waypoint, yaw):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        point_a = [x + np.cos(yaw), y +  np.sin(yaw)]
        point_b = [x, y]
        point_c = waypoint

        angle = self.angle_between_points(point_a, point_b, point_c)

        return angle

    def adjust_trajectory(self):
        if (self.state != 'adjusting'):
            self.state = 'adjusting'
            angle_plus = self.calc_angle_waypoint(self.path[self.path_index], self.yaw + np.pi/60)
            angle_minus = self.calc_angle_waypoint(self.path[self.path_index], self.yaw - np.pi/60)
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

    def request_path(self):
        print('requesting path')
        self.future = self.path_client.call_async(self.req)
        

    def odom_callback(self, msg):
        self.odom = msg
        if (self.state == 'plotting'):
            print('plotting')
            return
        if (len(self.path) == 0):
            return
        self.update_trajectory_record()
        if (self.path_index < len(self.path) and not self.waiting_for_path):
            qtrn = self.qtrn_msg_to_array(self.odom.pose.pose.orientation)
            self.yaw = self.qtrn_yaw(qtrn)
            angle = self.calc_angle_waypoint(self.path[self.path_index], self.yaw)
            if (self.is_at_waypoint(self.path[self.path_index])):
                    self.stop()
                    self.path_index += 1
            else:
                if (angle > np.pi/30):
                    self.adjust_trajectory()
                else:
                    self.go_straight()
        else:
            self.stop()
            if (not self.waiting_for_path):
                self.request_path()
                self.waiting_for_path =  True
                
            if (self.future.done()):
                try:
                    response = self.future.result()
                except Exception as e:
                    print('service call failed')
                else:
                    if(response.success):
                        self.waiting_for_path = False
                        self.path_index = 0
                    else:
                        print('response not success')
                        self.map_plot()

    def map_callback(self, msg):
        self.map_path = msg.data

    def waypoints_callback(self, msg):
        arr = np.array(msg.data)
        self.path = arr.reshape(int(len(arr)/2), 2)
        print(self.path)

        


def main(args=None):
    rclpy.init(args=args)
    twister = Twister()
    rclpy.spin(twister)
    twister.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()