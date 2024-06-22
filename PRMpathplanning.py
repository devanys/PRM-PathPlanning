import random
import math
import heapq
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageOps, ImageDraw

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.edges = []

    def add_edge(self, node):
        self.edges.append(node)

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def is_in_obstacle(x, y, inflated_obstacle_map):
    return inflated_obstacle_map[int(y), int(x)] == 0

def line_in_obstacle(x0, y0, x1, y1, inflated_obstacle_map):
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    line_points = list(bresenham(x0, y0, x1, y1))
    for x, y in line_points:
        if inflated_obstacle_map[y, x] == 0:
            return True
    return False

def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
            
def generate_random_nodes(num_nodes, x_max, y_max, inflated_obstacle_map):
    nodes = []
    while len(nodes) < num_nodes:
        x = random.uniform(0, x_max)
        y = random.uniform(0, y_max)
        if not is_in_obstacle(x, y, inflated_obstacle_map):
            nodes.append(Node(x, y))
    return nodes

def connect_nodes(nodes, k, inflated_obstacle_map):
    for node in nodes:
        distances = []
        for other_node in nodes:
            if node != other_node and not line_in_obstacle(node.x, node.y, other_node.x, other_node.y, inflated_obstacle_map):
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

def plot_prm(nodes, path=None, obstacle_map=None):
    if obstacle_map is not None:
        plt.imshow(obstacle_map, cmap='gray', origin='lower')

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
x_max = 1200
y_max = 800
k = 5

image_path = 'Obsticle.png'
original_image = Image.open(image_path).convert('L')
obstacle_map = np.array(original_image)

inflated_obstacle_map = ImageOps.expand(original_image, border=5, fill=255) 
draw = ImageDraw.Draw(inflated_obstacle_map)
for y in range(obstacle_map.shape[0]):
    for x in range(obstacle_map.shape[1]):
        if obstacle_map[y, x] == 0:
            draw.rectangle([x, y, x + 10, y + 10], fill=0)

inflated_obstacle_map = np.array(inflated_obstacle_map)

nodes = generate_random_nodes(num_nodes, x_max, y_max, inflated_obstacle_map)
connect_nodes(nodes, k, inflated_obstacle_map)

start = nodes[0]
goal = nodes[-1]
path = a_star(start, goal)

if path:
    print("Path found:")
    for node in path:
        print(f"Node: ({node.x}, {node.y})")
else:
    print("No path found")

plot_prm(nodes, path, obstacle_map)
