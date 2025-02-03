#!/usr/bin/python3
import torch
from torch import nn
import heapq

class Nvidia_Model(nn.Module):
    def __init__(self):
        super().__init__()
        
        self.layer1 = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.BatchNorm2d(24),
            nn.ReLU(),
            nn.MaxPool2d(2)
        )

        self.layer2 = nn.Sequential(
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.BatchNorm2d(36),
            nn.ReLU(),
            nn.MaxPool2d(2)
        )
        
        self.layer3 = nn.Sequential(
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.BatchNorm2d(48),
            nn.ReLU(),
            nn.MaxPool2d(2)
        )

        self.layer4 = nn.Sequential(
            nn.Conv2d(48, 64, kernel_size=1),
            nn.BatchNorm2d(64),
            nn.ReLU()
        )

        self.layer5 = nn.Sequential(
            nn.Conv2d(64, 64, kernel_size=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=1, stride=1)
        )
        
        # Calculate the number of features in the flattened output
        # Assuming the output shape from layer5 is bx64x4x8
        # n_features = 64 * 4 * 8
        
        self.fc1 = nn.Linear(256, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        self.fc4 = nn.Linear(10, 4)  # Speed & Steering Angle
        
        self.relu = nn.ReLU()
        self.elu = nn.ELU()

    def forward(self, x):
        out = self.layer1(x)
        out = self.layer2(out)
        out = self.layer3(out)
        out = self.layer4(out)
        out = self.layer5(out)
        out = out.view(out.size(0), -1)  # Flatten the output from conv layers
        out = self.relu(self.fc1(out))
        out = self.elu(self.fc2(out))
        out = self.elu(self.fc3(out))
        out = self.fc4(out)
        return out


def dijkstra_shortest_path(graph, start_node, end_node):
    distances = {node: float('inf') for node in graph.nodes()}
    distances[start_node] = 0
    previous_nodes = {node: None for node in graph.nodes()}
    priority_queue = [(0, start_node)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node == end_node:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = previous_nodes[current_node]
            return path[::-1]

        for neighbor in graph.neighbors(current_node):
            distance = current_distance + 1  # Assuming uniform edge weights
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    raise ValueError("No path found between the nodes.")

# class Nvidia_Model_GPS(Nvidia_Model):
#     def __init__(self, graph):
#         super().__init__()
#         self.graph = graph

#     def forward(self, x, start_gps=None, end_gps=None):
#         # Perform forward pass through convolutional layers
#         out = super().forward(x)

#         # If start_gps and end_gps are provided, find the shortest path
#         if start_gps is not None and end_gps is not None:
#             start_node = self.find_nearest_node(start_gps)
#             end_node = self.find_nearest_node(end_gps)
#             path = dijkstra_shortest_path(self.graph, start_node, end_node)
#             return path

#         # Otherwise, return the output from the neural network
#         return out

#     def find_nearest_node(self, gps_coords):
#         # Find the nearest node in the graph based on GPS coordinates
#         nearest_node = None
#         min_distance = float('inf')

#         for node, attrs in self.graph.nodes(data=True):
#             node_coords = attrs['pos']
#             distance = ((gps_coords[0] - node_coords[0]) ** 2 + (gps_coords[1] - node_coords[1]) ** 2) ** 0.5
#             if distance < min_distance:
#                 min_distance = distance
#                 nearest_node = node

#         return nearest_node


class Nvidia_Model_GPS(Nvidia_Model):
    def __init__(self, graph):
        super().__init__()
        self.graph = graph

    def forward(self, x, start_gps=None, end_gps=None):
        # Perform forward pass through convolutional layers
        out = super().forward(x)

        # If start_gps and end_gps are provided, find the shortest path
        if start_gps is not None and end_gps is not None:
            start_node = self.find_nearest_node(start_gps)
            end_node = self.find_nearest_node(end_gps)
            path = dijkstra_shortest_path(self.graph, start_node, end_node)
            return path  # Sau orice altceva necesar pentru inferență

        # Otherwise, return the output from the neural network (direction, speed, etc.)
        return out

    def find_nearest_node(self, gps_coords):
        # Find the nearest node in the graph based on GPS coordinates
        nearest_node = None
        min_distance = float('inf')

        for node, attrs in self.graph.nodes(data=True):
            node_coords = attrs['pos']
            distance = ((gps_coords[0] - node_coords[0]) ** 2 + (gps_coords[1] - node_coords[1]) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_node = node

        return nearest_node
