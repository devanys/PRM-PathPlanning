import random
import math
import heapq
import matplotlib.pyplot as plt
from PIL import Image

# Definisi Node
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.edges = []
        
    def add_edge(self, node):
        self.edges.append(node)

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

# Placeholder function for checking if edge intersects with obstacle
def intersects_obstacle(node1, node2, floor_plan):
    # Convert node coordinates to pixel coordinates
    x1, y1 = int(node1.x), int(node1.y)
    x2, y2 = int(node2.x), int(node2.y)
    
    # Bresenham's line algorithm to get all points between node1 and node2
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    
    while True:
        # Check if the current point is within the bounds of the floor plan
        if 0 <= x1 < floor_plan.width and 0 <= y1 < floor_plan.height:
            # Get pixel color (assuming RGB mode, check for black color)
            pixel_color = floor_plan.getpixel((x1, y1))
            if pixel_color == (0, 0, 0):  # Black color (obstacle)
                return True  # Hit obstacle
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    
    # If all points between node1 and node2 are traversable
    return False

# Membuat PRM (Probabilistic Roadmap)
def generate_random_nodes(num_nodes, x_max, y_max, floor_plan):
    nodes = []
    max_attempts = 1000
    attempts = 0
    
    while len(nodes) < num_nodes and attempts < max_attempts:
        x = random.uniform(0, x_max)
        y = random.uniform(0, y_max)
        
        # Check if the generated node is not in an obstacle (black) area
        if not intersects_obstacle(Node(x, y), Node(x, y), floor_plan):
            nodes.append(Node(x, y))
        
        attempts += 1
    
    return nodes

def connect_nodes(nodes, k, floor_plan):
    for node in nodes:
        distances = []
        for other_node in nodes:
            if node != other_node:
                edge_clear = not intersects_obstacle(node, other_node, floor_plan)
                if edge_clear:
                    distances.append((distance(node, other_node), other_node))
        distances.sort(key=lambda x: x[0])
        for d, other_node in distances[:k]:
            if not intersects_obstacle(node, other_node, floor_plan):
                node.add_edge(other_node)
                other_node.add_edge(node)  # Assuming undirected graph

# Algoritma A* untuk Pencarian Jalur
def a_star(start, goal, nodes):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {start: None}
    g_score = {node: float('inf') for node in nodes}
    g_score[start] = 0
    f_score = {node: float('inf') for node in nodes}
    f_score[start] = distance(start, goal)
    
    while open_list:
        _, current = heapq.heappop(open_list)
        
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        
        for neighbor in current.edges:
            tentative_g_score = g_score[current] + distance(current, neighbor)
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + distance(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))
    
    return None

# Visualisasi Jalur dengan Peta Lantai tanpa menabrak obstacle
def plot_prm_with_floor_plan(nodes, start, goal, floor_plan, path=None):
    # Plot floor plan
    plt.figure(figsize=(8, 6))
    plt.imshow(floor_plan, extent=[0, x_max, 0, y_max])
    
    # Plot edges between nodes (avoiding obstacle edges)
    for node in nodes:
        for edge in node.edges:
            if not intersects_obstacle(node, edge, floor_plan):
                plt.plot([node.x, edge.x], [node.y, edge.y], 'k-', lw=0.5)
    
    # Plot nodes
    plt.scatter([node.x for node in nodes], [node.y for node in nodes], c='b')
    plt.scatter([start.x], [start.y], c='g', marker='o')
    plt.scatter([goal.x], [goal.y], c='r', marker='x')
    
    # Plot path if available
    if path:
        path_x = [node.x for node in path]
        path_y = [node.y for node in path]
        plt.plot(path_x, path_y, 'r-', lw=2)
    
    plt.xlim(0, x_max)
    plt.ylim(0, y_max)
    plt.gca().invert_yaxis()  # Invert y-axis to match image coordinates (origin at bottom-left)
    plt.title('Probabilistic Roadmap with A* Path Finding')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

# Parameters and execution
num_nodes = 100
x_max = 100
y_max = 100
k = 10

# Load floor plan image
floor_plan = Image.open('FloorPlan.png')

nodes = generate_random_nodes(num_nodes, x_max, y_max, floor_plan)
connect_nodes(nodes, k, floor_plan)

start = nodes[0]
goal = nodes[-1]

# Execute A* algorithm
path = a_star(start, goal, nodes)

if path:
    print("Path found:")
    for node in path:
        print(f"Node: ({node.x}, {node.y})")
else:
    print("No path found")

# Plot PRM with floor plan
plot_prm_with_floor_plan(nodes, start, goal, floor_plan, path)
