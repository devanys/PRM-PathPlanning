import random
import math
import heapq
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.edges = []
        
    def add_edge(self, node):
        self.edges.append(node)

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def generate_random_nodes(num_nodes, x_max, y_max):
    nodes = []
    for _ in range(num_nodes):
        x = random.uniform(0, x_max)
        y = random.uniform(0, y_max)
        nodes.append(Node(x, y))
    return nodes

def connect_nodes(nodes, k):
    for node in nodes:
        distances = []
        for other_node in nodes:
            if node != other_node:
                distances.append((distance(node, other_node), other_node))
        distances.sort(key=lambda x: x[0])
        for d, other_node in distances[:k]:
            node.add_edge(other_node)
            other_node.add_edge(node)

def a_star(start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in nodes}
    g_score[start] = 0
    f_score = {node: float('inf') for node in nodes}
    f_score[start] = distance(start, goal)
    
    while open_list:
        _, current = heapq.heappop(open_list)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in current.edges:
            tentative_g_score = g_score[current] + distance(current, neighbor)
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + distance(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))
    
    return None

def plot_prm(nodes, path=None):
    for node in nodes:
        for edge in node.edges:
            plt.plot([node.x, edge.x], [node.y, edge.y], 'k-', lw=0.5)
    
    if path:
        path_x = [node.x for node in path]
        path_y = [node.y for node in path]
        plt.plot(path_x, path_y, 'r-', lw=2)

    plt.scatter([node.x for node in nodes], [node.y for node in nodes], c='b')
    plt.scatter([start.x], [start.y], c='g', marker='o')
    plt.scatter([goal.x], [goal.y], c='r', marker='x')
    plt.show()

num_nodes = 100
x_max = 100
y_max = 100
k = 5

nodes = generate_random_nodes(num_nodes, x_max, y_max)
connect_nodes(nodes, k)

start = nodes[0]
goal = nodes[-1]
path = a_star(start, goal)

if path:
    print("Path found:")
    for node in path:
        print(f"Node: ({node.x}, {node.y})")
else:
    print("No path found")

plot_prm(nodes, path)
