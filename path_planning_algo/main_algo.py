import numpy as np
import torch
from ultis import constants as C

class UnitreeNavigationSystem:
    """
    Global Planner (A*) và Local Planner (MPPI).
    
    Nhiệm vụ:
    1. Nhận điểm đích -> Tính đường A*.
    2. Nhận trạng thái robot -> Tính vận tốc tối ưu (MPPI).
    3. Trả về lệnh v_x, v_y, omega.
    """

    def __init__(self, astar_planner, mppi_controller, device="cuda"):
        self.astar = astar_planner
        self.mppi = mppi_controller
        self.device = device

        # Trạng thái nội bộ
        self.global_path = []      # List các điểm [(x, y), ...]
        self.path_idx = 0          # Index của waypoint hiện tại robot đang đi tới
        self.is_goal_reached = False
        self.current_goal = None   # (x, y) của đích cuối cùng

        # Các tham số ngưỡng (Thresholds)
        self.goal_threshold = 0.2      # 20cm là coi như đến đích
        self.waypoint_threshold = 0.5  # 50cm là chuyển sang waypoint tiếp theo
        
        # Giới hạn vận tốc (Safety Limits) cho Unitree G1
        # [v_x_min, v_y_min, w_min]
        self.cmd_min = np.array([0.0, -0.35, -0.5]) 
        # [v_x_max, v_y_max, w_max]
        self.cmd_max = np.array([1.0, 0.35, 0.5])

    def set_goal(self, start_x, start_y, goal_x, goal_y):
        """
        Lập kế hoạch đường đi mới từ A* khi có mục tiêu mới.
        """
        # Nếu mục tiêu không đổi và chưa đến đích thì không cần tính lại (tùy chọn)
        if self.current_goal == (goal_x, goal_y) and not self.is_goal_reached:
            return True

        print(f"[NavSystem] Planning path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")
        
        # 1. Gọi A* Planner
        # Lưu ý: Class A* của bạn trả về rx, ry riêng biệt
        rx, ry = self.astar.planning(start_x, start_y, goal_x, goal_y)
        
        if not rx:
            print("[NavSystem] A* failed to find path!")
            return False

        # 2. Convert sang format [(x,y), ...] và đảo chiều (vì A* thường trả về từ Đích -> Start)
        self.global_path = list(zip(rx, ry))
        
        # 3. Reset trạng thái
        self.path_idx = 0
        self.is_goal_reached = False
        self.current_goal = (goal_x, goal_y)

        # 4. Nạp path vào MPPI (để tính toán chi phí bám đường)
        path_tensor = torch.tensor(self.global_path, dtype=torch.float32, device=self.device)
        self.mppi.global_path = path_tensor
        
        print(f"[NavSystem] Path found with {len(self.global_path)} waypoints.")
        return True

    def compute_velocity_command(self, curr_x, curr_y, curr_yaw):
        """
        Hàm chính cần gọi trong vòng lặp điều khiển.
        Input: Trạng thái robot hiện tại.
        Output: v_forward, v_lateral, yaw_rate
        """
        # 0. Nếu chưa có đường đi hoặc đã đến đích -> Dừng robot
        if not self.global_path or self.is_goal_reached:
            return 0.0, 0.0, 0.0

        # 1. Kiểm tra xem đã đến đích cuối cùng chưa
        dist_to_goal = np.hypot(self.current_goal[0] - curr_x, self.current_goal[1] - curr_y)
        if dist_to_goal < self.goal_threshold:
            print(f"[NavSystem] Reached Goal! Dist: {dist_to_goal:.2f}")
            self.is_goal_reached = True
            return 0.0, 0.0, 0.0

        # 2. Logic chuyển đổi Waypoint (Follow the carrot)
        # Lấy waypoint hiện tại đang nhắm tới
        if self.path_idx < len(self.global_path):
            wp_x, wp_y = self.global_path[self.path_idx]
            dist_to_wp = np.hypot(wp_x - curr_x, wp_y - curr_y)

            # Nếu đã đến gần waypoint này, chuyển sang cái tiếp theo
            if dist_to_wp < self.waypoint_threshold and self.path_idx < len(self.global_path) - 1:
                self.path_idx += 1
                # Cập nhật waypoint mới
                wp_x, wp_y = self.global_path[self.path_idx]

        # 3. Cấu hình mục tiêu cục bộ cho MPPI
        
        state_tensor = torch.tensor([[curr_x, curr_y, curr_yaw]], dtype=torch.float32, device=self.device)
        
        self.mppi.local_target = torch.tensor([wp_x, wp_y], dtype=torch.float32, device=self.device)

        # 4. Chạy tối ưu hóa MPPI
        # Hàm command của class MPPI trả về [vx, vy, omega]
        mppi_cmd_tensor = self.mppi.command(state_tensor)
        
        # Chuyển về numpy array (v_forward, v_lateral, yaw_rate)
        cmd = mppi_cmd_tensor.detach().cpu().numpy().squeeze()

        # 5. Giới hạn vận tốc
        # Đảm bảo robot không nhận lệnh chạy quá nhanh gây ngã
        cmd = np.clip(cmd, self.cmd_min, self.cmd_max)

        v_forward = cmd[0]
        v_lateral = cmd[1]
        yaw_rate  = cmd[2]

        return v_forward, v_lateral, yaw_rate

    def reset(self):
        self.global_path = []
        self.path_idx = 0
        self.is_goal_reached = False
        self.current_goal = None