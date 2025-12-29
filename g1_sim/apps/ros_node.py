# Copyright (c) 2025 VinRobotics. All rights reserved

import time
from threading import Thread

import rclpy
from isaacsim.core.utils.types import JointsState as JointsState
from omni.isaac.core.robots import Robot
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


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
            physics_dt: float = 0.005,
    ):
        rclpy.init()

        super().__init__(node_name="sim_ros_node")

        self.robot = robot
        self._robot_name = robot_name
        self.joint_states_topic = joint_states_topic
        self.joint_commands_topic = joint_commands_topic
        self.physics_dt = physics_dt

        self.joint_states = None
        self.joint_commands = None
        self.joint_states_publisher = None

        self.hand_joint_commands = None

    def init_publisher_subscription(self):
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Subscription /joint_commands
        self.create_subscription(
            JointState, self.joint_commands_topic, self.joint_commands_callback, qos_profile
        )

        # Publisher /joint_states
        self.joint_states_publisher = self.create_publisher(
            JointState, self.joint_states_topic, qos_profile
        )

    def init_publish_joint_states_thread(self):
        # Read joint_states
        self.joint_states_timer = self.create_timer(self.physics_dt, self.joint_states_callback)

        # publish joint_states in other thread
        self.publish_joint_states_thread = Thread(target=self.publish_joint_states, daemon=True)
        self.publish_joint_states_thread_running = True
        self.publish_joint_states_thread.start()

    def joint_commands_callback(self, msg: JointState):
        self.joint_commands = msg

    def joint_states_callback(self):
        try:
            sim_joints_state = self.robot.get_joints_state()
            if sim_joints_state is None:
                raise ValueError("Received None for joint positions.")

            # Convert isaacsim.core.utils.types.JointsState to sensor_msgs.msg._joint_state.JointState
            self.joint_states = sim_joints_state_to_ros_msg(self.robot.dof_names, sim_joints_state)
        except Exception as e:
            print(f"⚠️ Failed to get joint state for robot '{self._robot_name}': {e}")

    def publish_joint_states(self):
        next_time = time.perf_counter()
        while self.publish_joint_states_thread_running:
            next_time += self.physics_dt

            if self.joint_states is not None:
                self.joint_states_publisher.publish(self.joint_states)

            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def destroy_node(self):
        self.publish_joint_states_thread_running = False
        self.publish_joint_states_thread.join()
        super().destroy_node()
