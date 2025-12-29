# Copyright (c) 2025 VinRobotics. All rights reserved

import os

import omni
import omni.graph.core as og

# Direct control
import pxr

# ROS
# import rclpy
from isaacsim import SimulationApp
from isaacsim.core.api.controllers.articulation_controller import ArticulationController

# More imports that need to compare after we create the app
from omni.isaac.core import (
    SimulationContext,  # noqa E402
    World,
)
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils import (  # noqa E402
    prims,
)
from omni.isaac.core.utils.types import ArticulationAction

# ROS
# from ros_node import RosNode
# from sensor_msgs.msg import JointState

# Graph
from g1_robot_sim import graph_config
from g1_robot_sim.abstracts.graph import Graph
from g1_robot_sim.abstracts.sensor import IMU, RealSense ,KannalaBrandt

MODEL_DESCRIPTIONS_PATH = "/home/rtx3/cuctt14/LOCOMOTION/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0"

class Scene:
    def __init__(
            self,
            sim_app: SimulationApp,
            world: World = None,
            robot_name: str = "robot",
            robot_file_name: str = "robot",
            torso_joint_name: str = "torso_joint",
            target_prim_joint_name: str = "torso_joint",
            joint_states_topic: str = "/joint_states",
            joint_commands_topic: str = "/joint_commands",
            joint_names: list[str] = [],
            hand_joint_names: list[str] = [],
            physics_dt: float = 1.0 / 200,  # 200Hz
            rendering_dt: float = 1.0 / 60,  # 60Hz
            use_action_graph: bool = True,
            enable_gpu_dynamics: bool = True,
    ) -> None:
        self._asset_dir = MODEL_DESCRIPTIONS_PATH

        self.sim_app = sim_app
        self.world = world
        self._robot_name = robot_name
        self._robot_file_name = robot_file_name
        self.torso_joint_name = torso_joint_name
        self.target_prim_joint_name = target_prim_joint_name
        self.joint_states_topic = joint_states_topic
        self.joint_commands_topic = joint_commands_topic
        self.joint_names = joint_names
        self.hand_joint_names = hand_joint_names
        self.physics_dt = physics_dt
        self.rendering_dt = rendering_dt

        self.use_action_graph = use_action_graph
        if not self.use_action_graph and self.world is None:
            raise ValueError("Require World for using ArticulationAction.")

        self._robot = None
        self.ros_node = None
        self.articulation_controller = None

        self.simulation_context = SimulationContext(
            stage_units_in_meters=1.0, physics_dt=self.physics_dt, rendering_dt=self.rendering_dt
        )
        self.simulation_context.get_physics_context().enable_gpu_dynamics(enable_gpu_dynamics)

    def spawn_robot(
            self,
            robot_position: list[float] = [0.0, 0.0, 1.0],
            robot_orientation: list[float] = [1.0, 0.0, 0.0, 0.0],
    ) -> None:
        # robot_usd_file = (
        #     self._asset_dir
        #     + "/robots/"
        #     + self._robot_name
        #     + "/usd/"
        #     + self._robot_file_name
        #     + ".usd"
        # )
        robot_usd_file = "unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"
        
        robot_prim_path = "/" + self._robot_name
        prims.create_prim(
            robot_prim_path,
            "Xform",
            position=robot_position,
            orientation=robot_orientation,
            usd_path=robot_usd_file,
        )

        # if self.use_action_graph:
        #     graph_handle = self.create_robot_omnigraph(robot_name=self._robot_name)
        # else:
        #     self.create_articulation_controller(robot_name=self._robot_name)

    # def create_robot_omnigraph(
    #         self,
    #         robot_name: str,
    #         graph_name: str = "g1_robot_graph.yml",
    # ) -> Graph:
    #     config_dir = os.path.dirname(graph_config.__file__)
    #     config_path_ = os.path.join(config_dir, graph_name)

    #     robot_omni_graph = Graph(
    #         config_path=config_path_,
    #         prim_path=f"/{robot_name}",
    #         target_prim=f"/{robot_name}/joints/{self.target_prim_joint_name}",
    #         sim_joint_states_topic=self.joint_states_topic,
    #         sim_joint_commands_topic=self.joint_commands_topic,
    #     )
    #     og_key_create_node, og_keys_connect, og_keys_set_values = robot_omni_graph.get_graph()
    #     keys = og.Controller.Keys

    #     graph_path = f"/{robot_name}/RobotActionGraph"
    #     (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
    #         {
    #             "graph_path": graph_path,
    #             "evaluator_name": "execution",
    #             # "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
    #         },
    #         {
    #             keys.CREATE_NODES: og_key_create_node,
    #             keys.SET_VALUES: og_keys_set_values,
    #             keys.CONNECT: og_keys_connect,
    #         },
    #     )
    #     return graph_handle

    # def create_articulation_controller(
    #         self,
    #         robot_name: str,
    # ):
    #     self.articulation_controller = ArticulationController()
    #     robot_prim_path = "/" + robot_name
    #     articulation_view = ArticulationView(
    #         prim_paths_expr=robot_prim_path, name=robot_name + "_view"
    #     )
    #     self.world.scene.add(articulation_view)
    #     self.articulation_controller.initialize(articulation_view)

    # def spawn_objects(
    #         self,
    #         objects: dict,
    # ) -> None:
    #     for prim_name, value in objects.items():
    #         sub_dir = value.get("sub_dir", "./")
    #         usd_file = (
    #             self._asset_dir
    #             + "/objects/"
    #             + sub_dir
    #             + "/"
    #             + value["name"]
    #             + "/"
    #             + value["name"]
    #             + ".usd"
    #         )
    #         scale = value.get("scale", [1.0, 1.0, 1.0])
    #         prims.create_prim(
    #             "/" + prim_name,
    #             "Xform",
    #             position=value["position"],
    #             orientation=value["orientation"],
    #             scale=scale,
    #             usd_path=usd_file,
    #         )

    # def setup_joint_drives(self):
    #     self.stage = omni.usd.get_context().get_stage()

    #     self.joint_drives = {}

    #     for joint_name in self.joint_names + self.hand_joint_names:
    #         # Ignore torso_joint
    #         if joint_name == self.torso_joint_name:
    #             continue

    #         joint_path = f"/{self._robot_name}/joints/{joint_name}"
    #         joint_prim = self.stage.GetPrimAtPath(joint_path)

    #         if not joint_prim.IsValid():
    #             print(f"[WARN] Joint prim not found: {joint_path}")
    #             continue

    #         drive = pxr.UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    #         drive.CreateStiffnessAttr()
    #         drive.CreateDampingAttr()
    #         drive.CreateTargetPositionAttr()
    #         drive.CreateTargetVelocityAttr().Set(0.0)

    #         # Add joint friction and armature attributes
    #         joint_physics_api = pxr.PhysxSchema.PhysxJointAPI.Apply(joint_prim)
    #         joint_physics_api.CreateJointFrictionAttr().Set(0.01)
    #         joint_physics_api.CreateArmatureAttr().Set(0.1)

    #         self.joint_drives[joint_name] = drive

    #         if joint_name in self.joint_names:
    #             drive.GetStiffnessAttr().Set(0.0)
    #             drive.GetDampingAttr().Set(1.0)
    #             drive.GetMaxForceAttr().Set(1000000000000.0)
    #         elif joint_name in self.hand_joint_names:
    #             drive.GetStiffnessAttr().Set(1.0)
    #             drive.GetDampingAttr().Set(0.25)

    def get_robot(
            self,
            robot_name: str,
    ) -> Robot:
        _robot = self.simulation_context.scene.get_object(robot_name)
        if _robot is None:
            robot_prim_path = "/" + robot_name
            if self.simulation_context.stage.GetPrimAtPath(robot_prim_path).IsValid():
                print(
                    f"⚠️ Robot prim '{robot_prim_path}' exists but is not registered in scene. Attempting to wrap it."
                )
            else:
                print(f"⚠️ Robot prim '{robot_prim_path}' does not exists. Creating new one.")
            _robot = Robot(prim_path=robot_prim_path, name=robot_name)

            try:
                print(f"➕ Adding robot '{robot_name}' to scene...")
                self.simulation_context.scene.add(_robot)
            except Exception as e:
                print(f"🔁 Skipping add: {e}")

        return _robot

    # def init_communication_node(
    #         self,
    #         robot_name: str,
    #         ros_node=None,
    # ):
    #     self._robot = self.get_robot(robot_name)

    #     if ros_node is None:
    #         self.ros_node = RosNode(
    #             robot=self._robot,
    #             robot_name=robot_name,
    #             joint_states_topic=self.joint_states_topic,
    #             joint_commands_topic=self.joint_commands_topic,
    #             physics_dt=self.physics_dt,
    #         )
    #     else:
    #         self.ros_node = ros_node

    #     self.ros_node.init_publisher_subscription()
    #     self.ros_node.init_publish_joint_states_thread()

    # def parse_joint_commands(self, joint_commands: JointState):
    #     joint_indices = None
    #     joint_positions = None
    #     joint_velocities = None
    #     joint_efforts = None

    #     if self.ros_node:
    #         if joint_commands is not None:
    #             joint_indices = []
    #             if len(joint_commands.position) > 0:
    #                 joint_positions = []
    #             if len(joint_commands.velocity) > 0:
    #                 joint_velocities = []
    #             if len(joint_commands.effort) > 0:
    #                 joint_efforts = []

    #             for joint_idx, joint_name in enumerate(joint_commands.name):
    #                 if joint_name in self._robot.dof_names:
    #                     dof_idx = self._robot.dof_names.index(joint_name)
    #                     joint_indices.append(dof_idx)
    #                     if joint_idx < len(joint_commands.position):
    #                         joint_positions.append(joint_commands.position[joint_idx])
    #                     if joint_idx < len(joint_commands.velocity):
    #                         joint_velocities.append(joint_commands.velocity[joint_idx])
    #                     if joint_idx < len(joint_commands.effort):
    #                         joint_efforts.append(joint_commands.effort[joint_idx])

    #     return (
    #         joint_indices,
    #         joint_positions,
    #         joint_velocities,
    #         joint_efforts,
    #     )

    # def apply_articulation_action(
    #         self,
    #         joint_indices,
    #         joint_positions,
    #         joint_velocities,
    #         joint_efforts,
    # ):
    #     robot_action = ArticulationAction(
    #         joint_indices=joint_indices,
    #         joint_positions=joint_positions,
    #         joint_velocities=joint_velocities,
    #         joint_efforts=joint_efforts,
    #     )
    #     self.articulation_controller.apply_action(robot_action)
    #     applied_action = self.articulation_controller.get_applied_action()
    #     return applied_action

    def run(self):
        self.sim_app.update()
        # Necessary
        # Auto call simulation_context.play() after init
        self.simulation_context.initialize_physics()

        while self.sim_app.is_running():
            if not self.use_action_graph:
                # Detect reset then auto init physics and play sim
                if self.simulation_context.is_stopped():
                    self.simulation_context.step(render=False)
                    self.simulation_context.initialize_physics()

            # Run with a fixed step size
            self.simulation_context.step(render=True)

        #     if not self.use_action_graph:
        #         # Get input for ArticulationController
        #         # After simulation_context.step()
        #         rclpy.spin_once(self.ros_node, timeout_sec=0.001)

        #         # Body
        #         if self.ros_node.joint_commands is not None:
        #             (
        #                 joint_indices,
        #                 joint_positions,
        #                 joint_velocities,
        #                 joint_efforts,
        #             ) = self.parse_joint_commands(self.ros_node.joint_commands)

        #             # Apply articulation action
        #             if joint_indices and len(joint_indices) > 0:
        #                 self.apply_articulation_action(
        #                     joint_indices,
        #                     joint_positions,
        #                     joint_velocities,
        #                     joint_efforts,
        #                 )

        #         # Hand
        #         if self.ros_node.hand_joint_commands is not None:
        #             (
        #                 joint_indices,
        #                 joint_positions,
        #                 joint_velocities,
        #                 joint_efforts,
        #             ) = self.parse_joint_commands(self.ros_node.hand_joint_commands)

        #             # Apply articulation action
        #             if joint_indices and len(joint_indices) > 0:
        #                 self.apply_articulation_action(
        #                     joint_indices,
        #                     joint_positions,
        #                     joint_velocities,
        #                     joint_efforts,
        #                 )

        # if not self.use_action_graph:
        #     self.ros_node.destroy_node()
        #     rclpy.shutdown()

        self.simulation_context.stop()
        self.sim_app.close()

def create_rgbd_camera(
    cam_prim_path: str,
    cam_link_name: str,
    cam_position: list,  # [x, y, z]
    cam_orientation: list,  # [qw, qx, qy ,qz]
    cam_resolution: tuple = (640, 480),  # width, height
    cam_dimension: tuple = (640, 480),  # width, height
    cam_focal_length: tuple = (400.0, 400.0),  # fx, fy
    cam_optical_center: tuple = (320.0, 240.0),  # cx, cy
    cam_info_topic: str="/camera/camera/color/camera_info",
    cam_color_topic: str="/camera/camera/color/image_raw",
    cam_depth_topic: str="/camera/camera/aligned_depth_to_color/image_raw",
    cam_pointcloud_topic: str="/camera/camera/depth/color/points",
    cam_frequency: int = 30,
    cam_projection_type: str = "pinhole",
    cam_frame_id:  str = "camera_color_optical_frame",
    cam_graph_name: str = "camera_rgbd_graph.yml",
):
    return RealSense(
        cam_prim_path=cam_prim_path,
        cam_link_name=cam_link_name,
        cam_resolution=cam_resolution,
        cam_dimension=cam_dimension,
        cam_focal_length=cam_focal_length,
        cam_optical_center=cam_optical_center,
        cam_info_topic=cam_info_topic,
        cam_color_topic=cam_color_topic,
        cam_depth_topic=cam_depth_topic,
        cam_pointcloud_topic=cam_pointcloud_topic,
        cam_frequency=cam_frequency,
        cam_projection_types=cam_projection_type,
        cam_frame_id=cam_frame_id,
        cam_graph_name=cam_graph_name,
        cam_translation=cam_position,
        cam_orientation=cam_orientation,
    )

def create_fisheye_camera(
    cam_prim_path: str,
    cam_link_name: str,
    cam_position: list,  # [x, y, z]
    cam_orientation: list,  # [qw, qx, qy ,qz]
    cam_resolution: tuple = (640, 480),  # width, height
    cam_dimension: tuple = (640, 480),  # width, height
    cam_focal_length: tuple = (400.0, 400.0),  # fx, fy
    cam_optical_center: tuple = (320.0, 240.0),  # cx, cy
    cam_info_topic: str="/camera/fish_eye_camera/color/camera_info",
    cam_color_topic: str="/camera/fish_eye_camera/color/image_raw",
    cam_frequency: int = 30,
    cam_projection_type: str = "fisheyeKannalaBrandtK3",
    cam_frame_id: str = "camera_color_optical_frame",
    cam_graph_name: str = "camera_rgb_graph.yml",
    distortion_coefficients: tuple[float, float, float, float] = [0.05, 0.01, -0.003, -0.0005],
):
    return KannalaBrandt(
        cam_prim_path=cam_prim_path,
        cam_link_name=cam_link_name,
        cam_resolution=cam_resolution,
        cam_dimension=cam_dimension,
        cam_focal_length=cam_focal_length,
        cam_optical_center=cam_optical_center,
        cam_info_topic=cam_info_topic,
        cam_color_topic=cam_color_topic,
        cam_frequency=cam_frequency,
        cam_projection_types=cam_projection_type,
        cam_frame_id=cam_frame_id,
        cam_graph_name=cam_graph_name,
        cam_translation=cam_position,
        cam_orientation=cam_orientation,
        distortion_coefficients=distortion_coefficients,
    )

def create_imu(
        imu_prim_path: str,
        imu_link_name: str,
        imu_topic: str = "imu",
        imu_frame_id: str = "imu_link",
        imu_frequency: int = 60,
        imu_position: list[float] = [0.0, 0.0, 0.0],
        imu_orientation: list[float] = [1.0, 0.0, 0.0, 0.0],
):
    imu = IMU(
        imu_prim_path=imu_prim_path,
        imu_link_name=imu_link_name,
        imu_topic=imu_topic,
        imu_frame_id=imu_frame_id,
        imu_frequency=imu_frequency,
        imu_translation=imu_position,
        imu_orientation=imu_orientation,
    )
    return imu
