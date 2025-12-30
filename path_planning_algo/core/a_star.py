import math
import numpy as np
from .path_smoother import smooth_path_spline
from scipy.spatial import KDTree

class AStarPlanner:
    """
    A* Path Planner class (Global Planner)
    """

    def __init__(self, ox, oy, resolution, rr=0.37):
        """
        ox, oy: Obstacle coordinates (lists)
        resolution: Grid resolution [m]
        rr: Robot radius [m] (Safety margin)
        """
        self.resolution = resolution 
        self.rr = rr 

        # Determine minimum and maximum grid coordinates based on obstacles
        self.min_x, self.min_y = round(min(ox)), round(min(oy))
        self.max_x, self.max_y = round(max(ox)), round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        self.motion = self.get_motion_model()  # 8-connectivity motion
        self.calc_obstacle_map(ox, oy)         # Generate obstacle grid

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # grid index x
            self.y = y  # grid index y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy, smooth=True):
        """    
        Finds the path from start to goal using the A* algorithm
        
        Input:
            sx, sy: Start position [m]
            gx, gy: Goal position [m]
            smooth: Boolean to apply spline smoothing
        
        Output:
            rx, ry: Final path coordinates
        """

        # Convert world coordinates to grid indices
        goal_ix = self.calc_xy_index(gx, self.min_x)
        goal_iy = self.calc_xy_index(gy, self.min_y)

        # Validate if goal is reachable/valid
        if not self.verify_node(self.Node(goal_ix, goal_iy, 0, -1)):
            print("Goal is inside an obstacle or out of range")
            return [], []

        # Initialize start and goal nodes
        nstart = self.Node(self.calc_xy_index(sx, self.min_x),
                           self.calc_xy_index(sy, self.min_y), 0.0, -1)
        ngoal = self.Node(self.calc_xy_index(gx, self.min_x),
                          self.calc_xy_index(gy, self.min_y), 0.0, -1)

        # Define open and closed sets
        open_set, closed_set = dict(), dict() 
        open_set[self.calc_grid_index(nstart)] = nstart

        while len(open_set) > 0:
            # Select node with lowest estimated total cost: f = g + h
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            # Check if destination is reached
            if current.x == ngoal.x and current.y == ngoal.y:
                ngoal.parent_index = current.parent_index
                ngoal.cost = current.cost
                break

            del open_set[c_id]      # Remove from open set
            closed_set[c_id] = current  # Add to closed set

            # Explore 8 possible directions
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # Skip if node is invalid (collision/out of bounds) or already visited
                if not self.verify_node(node) or n_id in closed_set:
                    continue
                
                # Update node in open set if a cheaper path is found
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        # Backtrack to find the final path
        rx, ry = self.calc_final_path(ngoal, closed_set) 
        
        # Log path progress (sampled for brevity)
        for i, (x, y) in enumerate(zip(rx, ry)):
            if i % max(1, len(rx)//10) == 0 or i == len(rx)-1: 
                print(f"[{i:3d}] ({x:6.2f}, {y:6.2f})")
        print(f"Start: ({rx[0]:.2f}, {ry[0]:.2f}) → Goal: ({rx[-1]:.2f}, {ry[-1]:.2f})")
        
        # Smoothing with appropriate resolution 
        if smooth and len(rx) > 2:
            rx, ry = smooth_path_spline(rx, ry, smoothing_factor=0.3, resolution=0.1)
            for i, (x, y) in enumerate(zip(rx, ry)):
                if i % max(1, len(rx)//15) == 0 or i == len(rx)-1:
                    print(f"       [{i:4d}] ({x:6.2f}, {y:6.2f})")
        elif smooth and len(rx) <= 2:
            # Add intermediate points even for very short paths
            rx, ry = smooth_path_spline(rx, ry, smoothing_factor=0.1, resolution=0.1)  
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        """Reconstructs the path from goal to start via parent indices"""
        rx = [self.calc_grid_position(goal_node.x, self.min_x)]
        ry = [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        
        count = 0
        while parent_index != -1 and count < 1000:  # Safety limit to prevent infinite loops
            if parent_index not in closed_set:
                break
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
            count += 1
        
        return rx[::-1], ry[::-1]  # Reverse to get path from Start to Goal

    def calc_heuristic(self, n1, n2):
        """Calculates Euclidean distance between two nodes"""
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def calc_grid_position(self, index, min_pos):
        """Converts grid index to world position"""
        return index * self.resolution + min_pos

    def calc_xy_index(self, position, min_pos):
        """Converts world position to grid index"""
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        """Generates a unique ID for a grid node"""
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        """Validates if node is within bounds and not in collision"""
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
            """Builds a collision map using KDTree for fast distance queries"""
            self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
            
            if not ox:
                return
                
            # Initialize KDTree for efficient nearest neighbor search
            self.tree = KDTree(list(zip(ox, oy)))
            
            for ix in range(self.x_width):
                x = self.calc_grid_position(ix, self.min_x)
                for iy in range(self.y_width):
                    y = self.calc_grid_position(iy, self.min_y)
                    
                    # Query distance to the nearest obstacle point
                    dist, _ = self.tree.query([x, y], k=1)
                    
                    # Mark as obstacle if distance is within robot radius
                    if dist <= self.rr:
                        self.obstacle_map[ix][iy] = True

    @staticmethod
    def get_motion_model():
        """
        Defines the 8-directional motion model
        Returns: [dx, dy, step_cost]
        """
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]