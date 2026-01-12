import rclpy
from rclpy.node import Node
import numpy as np
import math
import matplotlib.pyplot as plt
import networkx as nx
from custom_interface.msg import WaypointList, WaypointPairs, AdjacencyMatrix


class GraphPublisher(Node):
    def __init__(self):
        super().__init__('graph_publisher')

        self.publisher_waypoints = self.create_publisher(
            WaypointList,
            '/waypoints',
            10
        )
        self.publisher_pairs = self.create_publisher(
            WaypointPairs,
            '/waypoint_pairs',
            10
        )
        self.publisher_matrix = self.create_publisher(
            AdjacencyMatrix,
            '/adjacency_matrix',
            10
        )
        

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        print("created stuff")
        
        r11 = (0.92, 8.01)
        r12 = (0.1,4.9)
        r13 = (-1.93, 1.34)
        r14 = (-3.56, 7.5)
        r15 = (-3.54, 4.89)
        r16 = (-5.03, 2.09)

        r21 = (0.44, -3.78)
        r22 = (0.86, -8.08)
        r23 = (-1.91, 0.01)
        r24 = (-2.01, 4.63)
        r25 = (-3.62, -8.25)
        r26 = (-4.71, -0.72)
        r27 = (-4.81, -4.73)

        r31 = (6.2, -6.63)
        r32 = (6.3, -8.08)
        r33 = (2.29, -8.11)

        r41 = (6.19, -5.58)
        r42 = (2.44, -1.09)
        r43 = (1.81, -3.57)

        c11 = (-6.26, 2.07)
        c12 = (-6.12, -0.77)
        c13 = (-8.31, 2.07)
        c14 = (-8.35, -0.72)

        c21 = (-6.26, -4.81)
        c22 = (-4.91, -8.27)
        c23 = (-7.97, -4.86)
        c24 = (-8.06, -8.25)
        
        c31 = (7.98, 4.19)
        c32 = (8.06, 0.65)
        c33 = (1.18, 3.94)
        c34 = (2.27, 0.44)

        self.waypoints = [
            r11,
            r12,
            r13,
            r14,
            r15,
            r16,
        ]
        """
            r21,
            r22,
            r23,
            r24,
            r25,
            r26,
            r27,
            r31,
            r32,
            r33,
            r41,
            r42,
            r43,
            c11,
            c12,
            c13,
            c14,
            c21,
            c22,
            c23,
            c24,
            c31,
            c32,
            c33,
            c34,
        ]
        """
        self.neighbor_pairs = [
            (0,1),(0,3),(1,2),(1,4),(2,4),(2,5),(3,4),(4,5),#R1
        ]
        """
            (6,7),(6,9),(7,9),(7,10),(8,9),(8,11),(9,11),(9,12),(9,10),(10,12),(11,12),#R2
            (13,14),(14,15),#R3
            (16,17),(16,18),(17,18),#R4
            (19,21),(20,23),(21,23),#C1
            (24,26),(25,27),(26,27),#C2
            (28,30),(28,29),(29,31),#C3
            (1,30),#R1C3
            (2,8),#R1R2
            (5,19),#R1C1
            (17,6),#R2R4
            (7,)
            (11,20),#R2C1

        ]
        """
        self.number_of_nodes = len(self.waypoints)
        self.start_node = 0#TODO
        self.end_node = 5#TODO

        self.adjacency_matrix = [
            [0,1,0,1,0,0],
            [1,0,1,0,1,0],
            [0,1,0,0,1,1],
            [1,0,0,0,1,0],
            [0,1,1,1,1,1],
            [0,0,1,0,1,0]
        ]

        self.draw_map()
    
    def timer_callback(self):
        msgw = WaypointList()
        for waypoint in self.waypoints:
            msgw.waypoints.append(waypoint[0])
            msgw.waypoints.append(waypoint[1])
        self.publisher_waypoints.publish(msgw)

        msgp = WaypointPairs()
        for pair in self.neighbor_pairs:
            msgp.pairs.append(pair[0])
            msgp.pairs.append(pair[1])
        self.publisher_pairs.publish(msgp)

        msgm = AdjacencyMatrix()
        for y in self.adjacency_matrix:
            for x in y:
                msgm.matrix.append(x)
        msgm.size = len(self.adjacency_matrix)
        self.publisher_matrix.publish(msgm)

    def draw_map(self):
        # --- Create and draw the graph ---
        G = nx.Graph()
        G.add_edges_from(self.neighbor_pairs)
        positions = {i: self.waypoints[i] for i in range(self.number_of_nodes)}

        plt.figure(figsize=(9, 6))
        nx.draw(G, pos=positions, with_labels=True, node_color='lightblue', node_size=600, font_weight='bold')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    graph_publisher = GraphPublisher()
    try:
        rclpy.spin(graph_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        graph_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()