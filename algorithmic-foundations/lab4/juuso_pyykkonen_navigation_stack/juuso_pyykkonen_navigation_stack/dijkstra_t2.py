from array import array
from importlib.resources import path
from venv import create
import numpy as np
from queue import Queue, PriorityQueue
from graph import graph
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class Dijkstra(Node):
    def __init__(self):
        super().__init__('dijkstra')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_policy)

        self.publisher_ = self.create_publisher(Float64MultiArray, '/waypoints', 10)

        self.waypoints =[[0,0], [3, 0.4], [3.4,-2.6], [5.8, -2.5], [6, 7], [7.8,7],
                        [8,-2.7], [7.5,-5], [3, -5.3], [3,-9.5], [6,-8], [0,-8],
                        [0,-5], [-2,-4], [-3,-1], [-5,2], [-1.8,2.5], [-1.4,4.6],
                        [0.5,4.5], [0.6,2.3], [3,2.3], [3,7], [-1.3,7],
                        [-6.7, 0], [-6,-3], [-4,-6], [-8,-7]]

        self.neighbor_pairs = [(0,1), (1,2), (2,3), (3,4), (3,6),
                        (4,5), (4,21), (21,22), (5,6), (6,7),
                        (7,8), (8,9), (9,10), (9,11), (11,12), (11,25), (11,26),
                        (12,13), (13,14), (14,15), (15, 23), (23, 24),
                        (24, 25), (24, 26), (25, 26), (15,16), (16,17),
                        (17,18), (18,19), (19,20), (20,21)]

        self.end_node = 26


    def path_to_coordinates(self, path, waypoints):
            coordinates = []
            for node in path:
                coordinates.append(waypoints[node])
            return coordinates

    def dijkstra_alg(self, adjacency_matrix, weight_matrix, start_node, end_node):

        visited = set()
        q = PriorityQueue()
        q.put((0, start_node, [start_node]))
        visited.add(start_node)

        while (not q.empty()):

            current_cost, current_node, current_path = q.get()

            if (current_node == end_node):
                print('Path finished')
                print('The path is: {}'.format(current_path))
                print('The total cost is:'.format(current_cost))
                break

            for neighbor_id, is_a_neighbor in enumerate(adjacency_matrix[current_node]):

                if (is_a_neighbor == 1):
                    if (neighbor_id not in visited):
                        visited.add(neighbor_id)
                        q.put((current_cost + weight_matrix[current_node][neighbor_id], neighbor_id, current_path + [neighbor_id]))

        return current_path

    def nearest_waypoint(self, position, waypoints):
        nearest_node = 0
        least_distance = 10000
        for i in range(0, len(waypoints)):
            point = [waypoints[i][0]- position[0], waypoints[i][1] - position[1]]
            distance = np.sqrt((np.power(point[0], 2) + np.power(point[1], 2)))
            if (distance < least_distance):
                least_distance = distance
                nearest_node = i
        return nearest_node

    def create_path_msg(self, coordinates):
        path_msg = Float64MultiArray()
        coordinates = np.array(coordinates).flatten()
        for coord in coordinates:
            path_msg.data.append(coord)
        return path_msg

    def odom_callback(self, msg):
        odom = msg
        pos = [odom.pose.pose.position.x, odom.pose.pose.position.y]
        start_node = self.nearest_waypoint(pos, self.waypoints)

        graph_map = graph(self.waypoints, self.neighbor_pairs)

        adjacency_matrix = graph_map[0]
        weight_matrix = graph_map[1]

        path = self.dijkstra_alg(adjacency_matrix, weight_matrix, start_node, self.end_node)
        path_coordinates = self.path_to_coordinates(path, self.waypoints)
        print(path_coordinates)

        path_msg = self.create_path_msg(path_coordinates)

        self.publisher_.publish(path_msg)




def main(args=None):
    rclpy.init(args=args)
    dijkstra = Dijkstra()
    rclpy.spin_once(dijkstra)
    dijkstra.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
