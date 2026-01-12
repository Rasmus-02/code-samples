import rclpy
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from rclpy.node import Node
import math
from custom_interface.msg import WaypointList, AdjacencyMatrix, Finish
from std_msgs.msg import Bool

from queue import Queue, PriorityQueue

class Dijkstra(Node):

    def __init__(self):
        super().__init__('dijkstra')
        self.waypoints = []
        self.adjacency_matrix = []
        self.start_node = 0
        self.end_node = 5
        self.dijkstra_path = []

        self.subscription_waypoints = self.create_subscription(
            WaypointList,
            '/waypoints',
            self.callback_waypoints,
            10
        )
        self.subscription_matrix = self.create_subscription(
            AdjacencyMatrix,
            '/adjacency_matrix',
            self.callback_matrix,
            10
        )
        self.subscription_calculate = self.create_subscription(
            Finish,
            '/has_finished',
            self.dijkstra,
            10
        )
        self.publisher_finished_list = self.create_publisher(
            WaypointList,
            '/dijkstra_waypoints',
            10
        )

    def get_weight(self, point_1, point_2):
        pos_1 = self.waypoints[point_1]
        pos_2 = self.waypoints[point_2]
        return math.dist(pos_1, pos_2)
    
    def dijkstra(self, msg):
        if not msg.finished or len(self.adjacency_matrix) == 0 or len(self.waypoints) == 0:
            return
        self.update_start_node(msg.position)
        visited = set()
        q = PriorityQueue()
        q.put((0, self.start_node, [self.start_node]))
        visited.add(self.start_node)

        print("-----------Dijkstra path calculation-------------")
        path_calculation_cost = 0
        while not q.empty():
            path_calculation_cost += 1
            current_cost, current_node, current_path = q.get()
            print(current_path) #uncomment this to see all paths considered by the algorithm

            if current_node == self.end_node:
                print("We are at the end node")
                print("The path taken was: {}".format(current_path))
                print(f"Total cost is: {current_cost:.3f}")
                print(f"Calculation steps {path_calculation_cost}")
                self.dijkstra_path = current_path
                break
            for neighbor_id, is_a_neighbor in enumerate(self.adjacency_matrix[current_node]):

                if is_a_neighbor == 1 :

                    if neighbor_id not in visited:
                        visited.add(neighbor_id)

                        q.put((current_cost + self.get_weight(current_node, neighbor_id), neighbor_id, current_path + [neighbor_id]))
        
        msg = WaypointList()
        for index in self.dijkstra_path:
            msg.waypoints.append(self.waypoints[index][0])
            msg.waypoints.append(self.waypoints[index][1])
        self.publisher_finished_list.publish(msg)
        self.a_star() # Run A* for comparison

    def a_star(self):
        """A* pathfinding using the same data."""
        if len(self.adjacency_matrix) == 0 or len(self.waypoints) == 0:
            print("A*: Missing data (no adjacency matrix or waypoints).")
            return

        visited = set()
        q = PriorityQueue()
        q.put((0 + self.heuristic(self.start_node), 0, self.start_node, [self.start_node]))
        visited.add(self.start_node)

        print("\n\n\n---------A* path calculation------------")
        path_calculation_cost = 0
        while not q.empty():
            path_calculation_cost += 1
            est_total, current_cost, current_node, current_path = q.get()
            print(current_path)

            if current_node == self.end_node:
                print("----------")
                print(f"Path: {current_path}")
                print(f"Total cost: {current_cost:.3f}")
                print(f"Calculation steps {path_calculation_cost}")
                return

            for neighbor_id, is_a_neighbor in enumerate(self.adjacency_matrix[current_node]):
                if is_a_neighbor == 1 and neighbor_id not in visited:
                    visited.add(neighbor_id)
                    new_cost = current_cost + self.get_weight(current_node, neighbor_id)
                    est_total = new_cost + self.heuristic(neighbor_id)
                    q.put((est_total, new_cost, neighbor_id, current_path + [neighbor_id]))

    def update_start_node(self, robot_position):
        current_position = (robot_position[0], robot_position[1])
        closest_node_index = 0
        closest_node_distance = float('inf')
        
        for i in range(len(self.waypoints)):
            current_node_distance = math.dist(current_position, self.waypoints[i])
            if current_node_distance < closest_node_distance:
                closest_node_distance = current_node_distance
                closest_node_index = i
        self.start_node = closest_node_index

    def heuristic(self, node):
        return math.dist(self.waypoints[node], self.waypoints[self.end_node])

    def callback_waypoints(self, msg):
        data = msg.waypoints
        point_list = []
        for i in range(0, len(data), 2):
            point = (data[i], data[i+1])
            point_list.append(point)
        self.waypoints = point_list
    
    def callback_matrix(self, msg):
        data = msg
        matrix_list = []
        for y in range(data.size):
            inside = []
            for x in range(data.size):
                inside.append(data.matrix[y*data.size + x])
            matrix_list.append(inside)
        self.adjacency_matrix = matrix_list


def main(args=None):
    rclpy.init(args=args)
    dijkstra = Dijkstra()
    try:
        rclpy.spin(dijkstra)
    except KeyboardInterrupt:
        pass
    finally:
        dijkstra.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()