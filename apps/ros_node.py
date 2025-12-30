

import time
from threading import Thread

import numpy as np
import torch

import rclpy
from isaacsim.core.utils.types import JointsState as JointsState
from omni.isaac.core.robots import Robot
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState, Imu


def quat_apply_inverse(quat: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """Apply an inverse quaternion rotation to a vector.

    Args:
        quat: The quaternion in (w, x, y, z). Shape is (..., 4).
        vec: The vector in (x, y, z). Shape is (..., 3).

    Returns:
        The rotated vector in (x, y, z). Shape is (..., 3).
    """
    # store shape
    shape = vec.shape
    # reshape to (N, 3) for multiplication
    quat = quat.reshape(-1, 4)
    vec = vec.reshape(-1, 3)
    # extract components from quaternions
    xyz = quat[:, 1:]
    t = xyz.cross(vec, dim=-1) * 2
    return (vec - quat[:, 0:1] * t + xyz.cross(t, dim=-1)).view(shape)


def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def sim_joints_state_to_ros_msg(joint_names: list[str], sim_joints_state: JointsState) -> JointState:
    msg = JointState()

    ros_time = rclpy.clock.Clock().now().to_msg()
    msg.header.stamp = ros_time

    msg.name = joint_names
    msg.position = sim_joints_state.positions.tolist() if sim_joints_state.positions is not None else []
    msg.velocity = sim_joints_state.velocities.tolist() if sim_joints_state.velocities is not None else []
    msg.effort = sim_joints_state.efforts.tolist() if sim_joints_state.efforts is not None else []

    return msg


class RosNode(Node):
    def __init__(
            self,
            robot: Robot = None,
            robot_name: str = "robot",
            joint_states_topic: JointState = "/joint_states",
            joint_commands_topic: JointState = "/joint_commands",
            imu_topic: Imu = "/imu",
            physics_dt: float = 0.005,
    ):
        rclpy.init()

        super().__init__(node_name="sim_ros_node")

        self.robot = robot
        self._robot_name = robot_name
        self.joint_states_topic = joint_states_topic
        self.joint_commands_topic = joint_commands_topic
        self.imu_topic = imu_topic
        self.physics_dt = physics_dt

        self.joint_states = None
        self.joint_commands = None
        self.joint_states_publisher = None

        self.quat = None
        self.ang_vel = None
        self.counter = 0
        self.period = 1.2
        self.cmd_scale = np.array([2.0, 2.0, 0.25])
        self.max_cmd = np.array([0.8, 0.5, 1.57])
        self.dof_vel_scale = 0.05
        self.ang_vel_scale = 0.25
        self.dof_pos_scale = 1.0
        self.action_scale = 0.25
        self.obs = np.zeros(47)
        self.action = np.zeros(12)
        self.policy = torch.jit.load("/home/ros2_nh/ros2_ws/src/g1_sim/src/g1_robot_sim/assets/policy/motion.pt")
        
        self.hand_joint_commands = None

    def init_publisher_subscription(self):
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Subscription /joint_commands
        self.create_subscription(
            JointState, self.joint_commands_topic, self.joint_commands_callback, qos_profile
        )

        # Subscription /imu
        self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile
        )

        # Publisher /joint_states
        self.joint_states_publisher = self.create_publisher(
            JointState, self.joint_states_topic, qos_profile
        )

        # Publisher /joint_commands
        self.joint_commands_publisher = self.create_publisher(
            JointState, self.joint_commands_topic, qos_profile
        )

    def init_publish_joint_states_thread(self):
        # Read joint_states
        self.joint_states_timer = self.create_timer(self.physics_dt, self.joint_states_callback)
        self.joint_commands_timer = self.create_timer(self.physics_dt, self.infer_callback)

        # publish joint_states in other thread
        self.publish_joint_states_thread = Thread(target=self.publish_joint_states, daemon=True)
        self.publish_joint_states_thread_running = True
        self.publish_joint_states_thread.start()

        # publish joint_commands in other thread
        self.publish_joint_commands_thread = Thread(target=self.publish_joint_commands, daemon=True)
        self.publish_joint_commands_thread_running = True
        self.publish_joint_commands_thread.start()

    def joint_commands_callback(self, msg: JointState):
        self.joint_commands = msg

    def joint_states_callback(self):
        try:
            self.get_logger().info("Getting joint state for robot '{}'".format(self._robot_name))
            sim_joints_state = self.robot.get_joints_state()
            if sim_joints_state is None:
                raise ValueError("Received None for joint positions.")

            # Convert isaacsim.core.utils.types.JointsState to sensor_msgs.msg._joint_state.JointState
            self.joint_states = sim_joints_state_to_ros_msg(self.robot.dof_names, sim_joints_state)
        except Exception as e:
            print(f"⚠️ Failed to get joint state for robot '{self._robot_name}': {e}")

    def imu_callback(self, msg: Imu):
        self.get_logger().info("=" * 100)
        self.get_logger().info("IMU data received")
        self.quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.gravity_orientation = get_gravity_orientation(self.quat) 
        self.ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def infer_callback(self):
        if (
            self.quat is not None and
            self.ang_vel is not None
            and self.joint_states is not None
        ):
            self.counter += 1
            count = self.counter * self.physics_dt
            phase = count % self.period / self.period
            sin_phase = np.sin(2 * np.pi * phase)
            cos_phase = np.cos(2 * np.pi * phase)

            root_link_quat_w = torch.tensor(self.quat)
            gravity_vec_w = torch.tensor([0.0, 0.0, -1.0])
            gravity_vec_b = quat_apply_inverse(root_link_quat_w, gravity_vec_w).numpy()
            
            # Prepare obs
            num_actions = 12
            idxs = [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18]
            self.ang_vel = np.array(self.ang_vel, dtype=np.float32)
            self.obs[:3] = self.ang_vel * self.ang_vel_scale
            self.obs[3:6] = gravity_vec_w.numpy()
            self.obs[6:9] = np.array([0.1, 0.0, 0.0]) * self.cmd_scale * self.max_cmd  # TODO
            self.obs[9 : 9 + num_actions] = np.array(self.joint_states.position)[idxs] * self.dof_pos_scale
            self.obs[9 + num_actions : 9 + num_actions * 2] = np.array(self.joint_states.velocity)[idxs] * self.dof_vel_scale
            self.obs[9 + num_actions * 2 : 9 + num_actions * 3] = self.action
            self.obs[9 + num_actions * 3] = sin_phase
            self.obs[9 + num_actions * 3 + 1] = cos_phase

            # Get the action from the policy network
            obs_tensor = torch.from_numpy(self.obs).unsqueeze(0).float()
            self.action = self.policy(obs_tensor).detach().numpy().squeeze()

            position = np.zeros(29)
            target_dof_pos = self.action * self.action_scale
            position[idxs] = target_dof_pos

            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = list(self.robot.dof_names)

            cmd_msg.position = position.tolist()
            cmd_msg.velocity = [0.0] * 29  # singular
            cmd_msg.effort   = list(self.joint_states.effort)    # singular

            # self.joint_commands_publisher.publish(cmd_msg)
            self.joint_commands = cmd_msg
            

    def publish_joint_states(self):
        next_time = time.perf_counter()
        while self.publish_joint_states_thread_running:
            next_time += self.physics_dt

            if self.joint_states is not None:
                self.joint_states_publisher.publish(self.joint_states)

            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def publish_joint_commands(self):
        next_time = time.perf_counter()
        while self.publish_joint_commands_thread_running:
            next_time += self.physics_dt

            if self.joint_commands is not None:
                self.joint_commands_publisher.publish(self.joint_commands)

            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def destroy_node(self):
        self.publish_joint_states_thread_running = False
        self.publish_joint_commands_thread_running = False
        self.publish_joint_states_thread.join()
        self.publish_joint_commands_thread.join()
        super().destroy_node()
