# Copyright (c) 2025 VinRobotics. All rights reserved

import time

import rclpy
from isaacsim.core.utils.types import JointsState as JointsState
from omni.isaac.core.robots import Robot
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# ROS
from ros_node import RosNode
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowCmd, LowState
from src.g1_robot_sim.abstracts.sensor import IMU


def sim_joints_state_and_imu_to_lowstate(
        joint_names: list[str], sim_joints_state: JointsState, imu: IMU = None,
        joint_state_map: dict = None
) -> LowState:
    msg = LowState()
    msg.version = [1, 0]
    msg.mode_pr = 0
    msg.mode_machine = 0
    msg.tick = 0  # TODO

    # Only msg
    if imu is not None:
        imu_data = imu.get_current_frame()
        if imu_data:
            msg.imu_state.quaternion = imu_data["orientation"].tolist()
            msg.imu_state.gyroscope = imu_data["ang_vel"].tolist()
            msg.imu_state.accelerometer = imu_data["lin_acc"].tolist()
            # Optional: compute RPY from quaternion if needed
            msg.imu_state.rpy = [
                0.0,
                0.0,
                0.0,
            ]  # TODO: compute if needed
            msg.imu_state.temperature = 25  # Replace if you have actual temp sensor
    else:
        # Fallback to placeholder
        msg.imu_state.quaternion = [0.0, 0.0, 0.0, 1.0]
        msg.imu_state.gyroscope = [0.0, 0.0, 0.0]
        msg.imu_state.accelerometer = [0.0, 0.0, 9.8]
        msg.imu_state.rpy = [0.0, 0.0, 0.0]
        msg.imu_state.temperature = 25

    target_joint_names = list(joint_state_map.keys())

    for i in range(len(msg.motor_state)):  # 35 motors
        motor = msg.motor_state[i]

        motor.q = 0.0
        motor.dq = 0.0
        motor.ddq = 0.0  # no acceleration info here
        motor.tau_est = 0.0  # no torque info here
        motor.temperature = [0, 0]
        motor.vol = 0.0
        motor.sensor = [0, 0]
        motor.motorstate = 0
        motor.reserve = [0, 0, 0, 0]

        if i < len(target_joint_names):
            joint_name = target_joint_names[i]

            if joint_name in joint_names:
                idx = joint_names.index(joint_name)

                motor.q = float(sim_joints_state.positions[idx])
                motor.dq = (
                    float(sim_joints_state.velocities[idx])
                    if sim_joints_state.velocities is not None
                    else 0.0
                )

                # save for make /lowcmd
                joint_state_map[joint_name] = {
                    "q": motor.q,
                    "dq": motor.dq,
                }

    # wireless_remote and reserve placeholders
    msg.wireless_remote = [0] * 40
    msg.reserve = [0] * 4
    msg.crc = 0  # TODO: compute CRC if required

    return msg


def lowcmd_to_ros_msg(
        joint_names: list[str], cmd: LowCmd, joint_state_map: dict = None, pos_cmd: bool = True
) -> JointState:
    msg = JointState()

    ros_time = rclpy.clock.Clock().now().to_msg()
    msg.header.stamp = ros_time

    msg.name = []
    msg.position = []
    msg.velocity = []
    msg.effort = []

    for i, (joint_name, state) in enumerate(joint_state_map.items()):
        if joint_name in joint_names:
            if i < len(cmd.motor_cmd):
                msg.name.append(joint_name)

                motor_cmd = cmd.motor_cmd[i]
                target_position = motor_cmd.q
                if pos_cmd:
                    msg.position.append(target_position)
                else:
                    kp = motor_cmd.kp
                    kd = motor_cmd.kd
                    target_torque = kp * (target_position - state["q"]) - kd * state["dq"]
                    msg.effort.append(target_torque)

    return msg


class UnitreeRosNode(RosNode):
    def __init__(
            self,
            robot: Robot = None,
            imu: IMU = None,
            robot_name: str = "robot",
            joint_states_topic: LowState = "/lowstate",
            joint_commands_topic: LowCmd = "/lowcmd",
            hand_joint_states_topic: LowState = "/inspire_hand/lowstate",
            hand_joint_commands_topic: LowCmd = "/inspire_hand/lowcmd",
            joint_names: list[str] = [],
            hand_joint_names: list[str] = [],
            physics_dt: float = 0.005,
    ):
        super().__init__(
            robot=robot,
            robot_name=robot_name,
            physics_dt=physics_dt,
        )

        self.imu = imu

        # Overwrite topic name and type
        self.joint_states_topic = joint_states_topic
        self.joint_commands_topic = joint_commands_topic

        # New topics for Hand
        self.hand_joint_states_topic = hand_joint_states_topic
        self.hand_joint_commands_topic = hand_joint_commands_topic

        # Store current joint state for Torque calculation
        self.joint_names = joint_names
        self.hand_joint_names = hand_joint_names
        self.joint_state_map = {
            name: {"q": 0.0, "kp": 500.0, "kd": 50.0} for name in self.joint_names
        }
        self.hand_joint_state_map = {
            name: {"q": 0.0, "kp": 0.0, "kd": 0.0} for name in self.hand_joint_names
        }

        # Body Joints
        self.joint_states = None
        self.joint_commands = None
        self.joint_states_publisher = None

        # Hand Joints
        self.hand_joint_states = None
        self.hand_joint_commands = None
        self.hand_joint_states_publisher = None

    def init_publisher_subscription(self):
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Subscription /lowcmd
        self.create_subscription(
            LowCmd, self.joint_commands_topic, self.joint_commands_callback, qos_profile
        )

        # Publisher /lowstate
        self.joint_states_publisher = self.create_publisher(
            LowState, self.joint_states_topic, qos_profile
        )

        # Subscription /inspire_hand/lowcmd
        self.create_subscription(
            LowCmd, self.hand_joint_commands_topic, self.hand_joint_commands_callback, qos_profile
        )

        # Publisher /inspire_hand/lowstate
        self.hand_joint_states_publisher = self.create_publisher(
            LowState, self.hand_joint_states_topic, qos_profile
        )

    def joint_commands_callback(self, msg: LowCmd):
        # Convert LowCmd to sensor_msgs.msg._joint_state.JointState
        self.joint_commands = lowcmd_to_ros_msg(
            self.robot.dof_names,
            msg,
            self.joint_state_map,
        )

    def hand_joint_commands_callback(self, msg: LowCmd):
        # Convert LowCmd to sensor_msgs.msg._joint_state.JointState
        self.hand_joint_commands = lowcmd_to_ros_msg(
            self.robot.dof_names,
            msg,
            self.hand_joint_state_map,
        )

    def joint_states_callback(self):
        try:
            sim_joints_state = self.robot.get_joints_state()
            if sim_joints_state is None:
                raise ValueError("Received None for joint positions.")

            # Convert isaacsim.core.utils.types.JointsState to LowState
            self.joint_states = sim_joints_state_and_imu_to_lowstate(
                joint_names=self.robot.dof_names,
                sim_joints_state=sim_joints_state,
                imu=self.imu,
                joint_state_map=self.joint_state_map,
            )

            self.hand_joint_states = sim_joints_state_and_imu_to_lowstate(
                joint_names=self.robot.dof_names,
                sim_joints_state=sim_joints_state,
                imu=None,
                joint_state_map=self.hand_joint_state_map,
            )
        except Exception as e:
            print(f"⚠️ Failed to get joint state for robot '{self._robot_name}': {e}")

    def publish_joint_states(self):
        next_time = time.perf_counter()
        while self.publish_joint_states_thread_running:
            next_time += self.physics_dt

            if self.joint_states is not None:
                self.joint_states_publisher.publish(self.joint_states)

            if self.hand_joint_states is not None:
                self.hand_joint_states_publisher.publish(self.hand_joint_states)

            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
