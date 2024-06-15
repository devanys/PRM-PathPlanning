import tkinter as tk
import numpy as np
import random

class PRM:
    def __init__(self, start, goal, num_nodes, map_size, obstacles):
        self.start = start
        self.goal = goal
        self.num_nodes = num_nodes
        self.map_size = map_size
        self.obstacles = obstacles
        self.nodes = [start, goal]
        self.edges = []

    def sample_nodes(self):
        while len(self.nodes) < self.num_nodes:
            x = random.uniform(0, self.map_size[0])
            y = random.uniform(0, self.map_size[1])
            if not self.in_obstacle((x, y)):
                self.nodes.append((x, y))

    def in_obstacle(self, node):
        x, y = node
        for obs in self.obstacles:
            if obs[0] <= x <= obs[2] and obs[1] <= y <= obs[3]:
                return True
        return False

    def add_edge(self, node1, node2):
        if node1 != node2 and node2 not in [n[1] for n in self.edges if n[0] == node1]:
            if not self.edge_intersects_obstacle(node1, node2):
                self.edges.append((node1, node2))

    def edge_intersects_obstacle(self, node1, node2):
        for obs in self.obstacles:
            if self.line_intersects_rect(node1, node2, obs):
                return True
        return False

    def line_intersects_rect(self, p1, p2, rect):
        x1, y1, x2, y2 = rect
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        A, B, C, D = (x1, y1), (x2, y1), (x2, y2), (x1, y2)
        return ccw(p1, A, B) != ccw(p2, A, B) and ccw(p1, C, D) != ccw(p2, C, D)

    def distance(self, node1, node2):
        return np.linalg.norm(np.array(node1) - np.array(node2))

    def find_path(self):
        self.sample_nodes()
        for node in self.nodes:
            neighbors = sorted(self.nodes, key=lambda n: self.distance(node, n))[1:6]
            for neighbor in neighbors:
                self.add_edge(node, neighbor)
        path = self.a_star()
        return path

    def a_star(self):
        open_set = {self.start}
        came_from = {}
        g_score = {node: float('inf') for node in self.nodes}
        g_score[self.start] = 0
        f_score = {node: float('inf') for node in self.nodes}
        f_score[self.start] = self.distance(self.start, self.goal)

        while open_set:
            current = min(open_set, key=lambda node: f_score[node])
            if current == self.goal:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)
            for _, neighbor in [e for e in self.edges if e[0] == current]:
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.distance(neighbor, self.goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        return []

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

class PRM_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Probabilistic Road Map Path Planning")
        self.create_widgets()
        self.map_size = (13, 13)
        self.start = (1, 1)
        self.goal = (12, 12)
        self.target = (6, 6)
        self.num_nodes = 100
        self.obstacles = [(2, 2, 3, 10), (4, 4, 10, 5), (4, 8, 10, 9)]
        self.ego_position = list(self.start)
        self.target_position = list(self.target)
        self.endpoint = list(self.goal)
        self.running = False
        self.path_ego = []
        self.path_target = []
        self.path_reference = []

    def create_widgets(self):
        self.canvas = tk.Canvas(self.root, width=500, height=500)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.button_frame = tk.Frame(self.root)
        self.button_frame.pack()
        self.plan_button = tk.Button(self.button_frame, text="Generate Path Planning", command=self.generate_path_planning)
        self.plan_button.pack(side=tk.LEFT)
        self.start_button = tk.Button(self.button_frame, text="Start Simulation", command=self.start_simulation)
        self.start_button.pack(side=tk.LEFT)
        self.quit_button = tk.Button(self.button_frame, text="Quit", command=self.root.quit)
        self.quit_button.pack(side=tk.LEFT)
        self.canvas.bind("<Configure>", self.on_resize)

    def on_resize(self, event):
        self.canvas_width = event.width
        self.canvas_height = event.height
        self.draw_paths()

    def generate_path_planning(self):
        prm_reference = PRM(tuple(self.start), tuple(self.goal), self.num_nodes, self.map_size, self.obstacles)
        self.path_reference = prm_reference.find_path()
        prm_ego = PRM(tuple(self.ego_position), tuple(self.target_position), self.num_nodes, self.map_size, self.obstacles)
        self.path_ego = prm_ego.find_path()
        prm_target = PRM(tuple(self.target_position), tuple(self.endpoint), self.num_nodes, self.map_size, self.obstacles)
        self.path_target = prm_target.find_path()
        self.nodes = prm_reference.nodes + prm_ego.nodes + prm_target.nodes
        self.edges = prm_reference.edges + prm_ego.edges + prm_target.edges
        self.draw_paths()

    def start_simulation(self):
        self.running = True
        self.run_simulation()

    def run_simulation(self):
        if self.running:
            self.plan_and_move()
            self.root.after(1000, self.run_simulation)  # update every second

    def plan_and_move(self):
        prm_ego = PRM(tuple(self.ego_position), tuple(self.target_position), self.num_nodes, self.map_size, self.obstacles)
        path_ego = prm_ego.find_path()
        if len(path_ego) > 1:
            self.ego_position = path_ego[1]

        prm_target = PRM(tuple(self.target_position), tuple(self.endpoint), self.num_nodes, self.map_size, self.obstacles)
        path_target = prm_target.find_path()
        if len(path_target) > 1:
            self.target_position = path_target[1]

        self.nodes = prm_ego.nodes + prm_target.nodes
        self.edges = prm_ego.edges + prm_target.edges
        self.path = path_ego + path_target
        self.draw_paths()

    def draw_paths(self):
        self.canvas.delete("all")
        x_scale = self.canvas_width / self.map_size[0]
        y_scale = self.canvas_height / self.map_size[1]

        for obs in self.obstacles:
            self.canvas.create_rectangle(obs[0]*x_scale, obs[1]*y_scale, obs[2]*x_scale, obs[3]*y_scale, fill='black')

        for edge in self.edges:
            (x1, y1), (x2, y2) = edge
            self.canvas.create_line(x1*x_scale, y1*y_scale, x2*x_scale, y2*y_scale, fill='blue', width=1, stipple='gray50')

        for node in self.nodes:
            self.canvas.create_oval(node[0]*x_scale-3, node[1]*y_scale-3, node[0]*x_scale+3, node[1]*y_scale+3, fill='blue')

        if self.path_reference:
            px, py = zip(*self.path_reference)
            for i in range(len(px)-1):
                self.canvas.create_line(px[i]*x_scale, py[i]*y_scale, px[i+1]*x_scale, py[i+1]*y_scale, fill='yellow', width=2)

        if self.path_ego:
            px, py = zip(*self.path_ego)
            for i in range(len(px)-1):
                self.canvas.create_line(px[i]*x_scale, py[i]*y_scale, px[i+1]*x_scale, py[i+1]*y_scale, fill='blue', width=2)

        if self.path_target:
            px, py = zip(*self.path_target)
            for i in range(len(px)-1):
                self.canvas.create_line(px[i]*x_scale, py[i]*y_scale, px[i+1]*x_scale, py[i+1]*y_scale, fill='red', width=2)

        self.canvas.create_oval(self.ego_position[0]*x_scale-5, self.ego_position[1]*y_scale-5, self.ego_position[0]*x_scale+5, self.ego_position[1]*y_scale+5, fill='green')
        self.canvas.create_oval(self.target_position[0]*x_scale-5, self.target_position[1]*y_scale-5, self.target_position[0]*x_scale+5, self.target_position[1]*y_scale+5, fill='blue')
        self.canvas.create_oval(self.endpoint[0]*x_scale-5, self.endpoint[1]*y_scale-5, self.endpoint[0]*x_scale+5, self.endpoint[1]*y_scale+5, fill='red')

    def stop_simulation(self):
        self.running = False

if __name__ == "__main__":
    root = tk.Tk()
    gui = PRM_GUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.stop_simulation)
    root.mainloop()
