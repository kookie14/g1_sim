from core import AStarPlanner, G1MPPIController
from main_algo import UnitreeNavigationSystem

# Khởi tạo
astar = AStarPlanner(ox, oy, resolution=0.1)
mppi = G1MPPIController(device="cuda", obstacles=obstacles)
nav = UnitreeNavigationSystem(astar, mppi)

# Đặt mục tiêu
nav.set_goal(start_x, start_y, goal_x, goal_y)

# Loop điều khiển
while simulation_app.is_running():
     # ... lấy state robot ...
     vx, vy, omega = nav.compute_velocity_command(curr_x, curr_y, curr_yaw)
     # ... điều khiển robot ...