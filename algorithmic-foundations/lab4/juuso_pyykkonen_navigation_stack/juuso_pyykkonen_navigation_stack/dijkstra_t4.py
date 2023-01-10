from importlib.resources import path
from venv import create
import numpy as np
from queue import Queue, PriorityQueue
from graph_t4 import graph
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.srv import NextWaypoint

class Dijkstra(Node):
    def __init__(self):
        super().__init__('dijkstra')

        self.srv = self.create_service(NextWaypoint, 'next_waypoint', self.next_node_callback)

        self.graph_map = graph()
        self.adjacency_matrix = self.graph_map[0]
        self.weight_matrix = self.graph_map[1]
        self.waypoints = self.graph_map[2]
        self.next_node_q = Queue()
        self.global_visited = set()

    def dijkstra_alg(self, start_node):
        
        visited = set()
        q = PriorityQueue()
        q.put((0, start_node, [start_node]))
        visited.add(start_node)
        self.global_visited.add(start_node)
        end_node = -1
        
        while (not q.empty()):

            current_cost, current_node, current_path = q.get()

            if (current_node == end_node):
                print('Path finished')
                print('The path is: {}'.format(current_path))
                print('The total cost is:'.format(current_cost))
                break

            for neighbor_id, is_a_neighbor in enumerate(self.adjacency_matrix[current_node]):

                if (is_a_neighbor == 1):
                    if (neighbor_id not in visited):
                        visited.add(neighbor_id)
                        q.put((current_cost + self.weight_matrix[current_node][neighbor_id], neighbor_id, current_path + [neighbor_id]))
                        if (neighbor_id not in self.global_visited):
                            end_node = neighbor_id
                            break
        # do not put the first node in the queue because the robot is
        # located in that point
        for i in range(1, len(current_path)):
            self.next_node_q.put(current_path[i])



    def nearest_waypoint(self, position):
        nearest_node = 0
        least_distance = 10000
        for i in range(0, len(self.waypoints)):
            point = [self.waypoints[i][0]- position[0], self.waypoints[i][1] - position[1]]
            distance = np.sqrt((np.power(point[0], 2) + np.power(point[1], 2)))
            if (distance < least_distance):
                least_distance = distance
                nearest_node = i
        print(nearest_node)
        return nearest_node

    def next_node_callback(self, request, response):
        if (len(self.global_visited) == len(self.waypoints)):
            response.all_visited = True
            response.x = request.x
            response.y = request.y
            print('all visited')
            return response
        if (self.next_node_q.empty()):
            current_node = self.nearest_waypoint([request.x, request.y])
            self.dijkstra_alg(current_node)

        next_node = self.next_node_q.get()
        next_waypoint = self.waypoints[next_node]
        response.x = float(next_waypoint[0])
        response.y = float(next_waypoint[1])
        print('The next node is: {}'.format(next_node))
        return response


def main(args=None):
    rclpy.init(args=args)
    dijkstra = Dijkstra()
    rclpy.spin(dijkstra)
    dijkstra.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
