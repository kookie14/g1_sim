import omni
import math
import numpy as np
import os
from typing import Optional

# Isaac Sim Core Imports
from isaacsim import SimulationApp
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)
simulation_app.update()

from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, prims, stage
import omni.usd
import omni.graph.core as og

#Direct control 
import pxr
from pxr import PhysxSchema
#Articulation controller
from omni.isaac.core.utils.types import ArticulationAction

from isaacsim.sensors.physics import IMUSensor

# Your Modules
from vr_robot_sim import graph_config, assets
from vr_robot_sim.abstracts.sensor import RealSense
from vr_robot_sim.abstracts.graph import Graph

# ROS 2
import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd ,LowState
from sensor_msgs.msg import JointState

# Enable necessary Isaac Sim extensions
extensions.enable_extension("omni.isaac.ros2_bridge")
extensions.enable_extension("isaacsim.sensors.physics.ui")

# Add environment
BACKGROUND_STAGE_PATH = "/background"
EVN_PATH = os.path.join(os.path.dirname(assets.__file__), "env/default_environment.usd")
stage.add_reference_to_stage(EVN_PATH, BACKGROUND_STAGE_PATH)

class LowCmdSubscriber(Node):
    def __init__(self):
        super().__init__('lowcmd_subscriber')

        self.joint_names = [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
            "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
            "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        ]
        self.hand_joint_names = [
            "R_pinky_proximal_joint", 
            "R_ring_proximal_joint", 
            "R_middle_proximal_joint",
            "R_index_proximal_joint",
            "R_thumb_proximal_yaw_joint", 
            "R_thumb_proximal_pitch_joint", 
            "L_pinky_proximal_joint", 
            "L_ring_proximal_joint", 
            "L_middle_proximal_joint", 
            "L_index_proximal_joint",
            "L_thumb_proximal_yaw_joint", 
            "L_thumb_proximal_pitch_joint" 
        ]
        self.joint_command_map = {name: {"q": 0.0, "kp": 0.0, "kd": 0.0} for name in self.joint_names}
        self.hand_joint_command_map = {name: {"q": 0.0, "kp": 0.0, "kd": 0.0} for name in self.hand_joint_names}        
        self.joint_state_map = {}

class Scene():
    def __init__(self, robot_name: str, 
                robot_position : Optional[np.ndarray] = None,
                robot_orientation : Optional[np.ndarray] = None )-> None:
        self._asset_dir = os.path.dirname(assets.__file__) 
        self._robot_name = robot_name
        self.joints_states = "/isaac_lowcmd"
        self.spawn_robot(robot_position,robot_orientation)
        self.t=0.0
    def spawn_robot(self,robot_position : Optional[np.ndarray] = None,
                    robot_orientation : Optional[np.ndarray] = None):
        robot_usd_file = self._asset_dir + "/robot/" + self._robot_name + "/" + self._robot_name +".usd"
        prims.create_prim(
         "/" + self._robot_name,
             "Xform",
             position=robot_position,
             orientation=robot_orientation,
             usd_path=robot_usd_file,
        )
        graph_handle = self.create_robot_omnigraph(robot_name=self._robot_name)
    def spawn_objects(self,objects: dict):
        for prim_name, value in objects.items() :
            usd_file = self._asset_dir + "/objects/" + value["name"] + "/" + value["name"] +".usd"
            scale = value.get("scale", [1.0, 1.0, 1.0]) 
            prims.create_prim(
                "/" + prim_name,
                "Xform",
                position=value["position"],
                orientation=value["orientation"],
                scale=scale,
                usd_path=usd_file,
            )  
    def create_robot_omnigraph(self,robot_name: str):
        config_dir = os.path.dirname(graph_config.__file__)
        graph_name = "g1_graph.yml"
        config_path_ = os.path.join(config_dir, graph_name)
        robot_omni_graph = Graph(
            config_path=config_path_,
            prim_path=f"/{robot_name}",
            sim_joint_states=self.joints_states ,
        )
        og_key_create_node, og_keys_connect, og_keys_set_values = robot_omni_graph.get_graph()
        keys = og.Controller.Keys

        graph_path = f"/{robot_name}/RobotActionGraph"
        (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                # "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                keys.CREATE_NODES: og_key_create_node,
                keys.SET_VALUES: og_keys_set_values,
                keys.CONNECT: og_keys_connect,
            },
        )
        return graph_handle
    def setup_direct_joint_control(self):
        self.stage = omni.usd.get_context().get_stage()
        self.joint_drives = {}
        for joint_name in [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
            "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
            "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            #"waist_yaw_joint","waist_roll_joint","waist_pitch_joint", 
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
            "R_pinky_proximal_joint", 
            "R_ring_proximal_joint", 
            "R_middle_proximal_joint",
            "R_index_proximal_joint",
            "R_thumb_proximal_yaw_joint", 
            "R_thumb_proximal_pitch_joint", 
            "L_pinky_proximal_joint", 
            "L_ring_proximal_joint", 
            "L_middle_proximal_joint", 
            "L_index_proximal_joint",
            "L_thumb_proximal_yaw_joint", 
            "L_thumb_proximal_pitch_joint"              
        ]:
            joint_path = f"/{self._robot_name}/joints/{joint_name}"
            joint_prim = self.stage.GetPrimAtPath(joint_path)
            if not joint_prim.IsValid():
                print(f"[WARN] Joint prim not found: {joint_path}")
                continue
            drive = pxr.UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive.CreateStiffnessAttr()
            drive.CreateDampingAttr()
            #drive.CreateMaxForceAttr().Set(100.0)
            drive.CreateTargetPositionAttr()
            drive.CreateTargetVelocityAttr().Set(0.0)
            # Add joint friction and armature attributes
            joint_physics_api = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
            joint_physics_api.CreateJointFrictionAttr().Set(0.0)
            joint_physics_api.CreateArmatureAttr().Set(0.05)            
            self.joint_drives[joint_name] = drive

    def run(self, ros_node: Optional[LowCmdSubscriber] = None):    
        from omni.isaac.core import SimulationContext

        simulation_app.update()
        simulation_context = SimulationContext(stage_units_in_meters=1.0, physics_dt=0.005) # timesteps per second = 200
        simulation_context.play()

        print("🔧 Running simulation loop...")
        while simulation_app.is_running():
            simulation_context.step(render=True)

            if ros_node:
                rclpy.spin_once(ros_node, timeout_sec=0.001)
                # Loop over all joint commands and apply them to the simulation
                for joint_name, cmd in ros_node.joint_command_map.items():
                    # Fetch the current joint drive based on joint name
                    drive = self.joint_drives.get(joint_name)
                    if drive:
                        # Get the state for the joint to calculate target position
                        #state = ros_node.joint_state_map.get(joint_name, {"q": 0.0, "dq": 0.0})

                        # Compute the target position using the PD-like control
                        #error = cmd["q"] - state["q"]
                        #target_position =  cmd["kp"] * error - cmd["kd"] * state["dq"]
                        #target_position =  state["q"] * 180 / math.pi
                        # Set the target position, stiffness (kp), and damping (kd) for the drive
                        #drive.GetTargetPositionAttr().Set(target_position)
                        drive.GetStiffnessAttr().Set(0.0)
                        drive.GetDampingAttr().Set(0.25)
                        drive.GetMaxForceAttr().Set(1000000000000.0)
                        #drive.CreateTargetVelocityAttr().Set(0.0)
                for joint_name, cmd in ros_node.hand_joint_command_map.items():
                    # Fetch the current joint drive based on joint name
                    drive = self.joint_drives.get(joint_name)
                    if drive:
                        # Get the state for the joint to calculate target position
                        #state = ros_node.joint_state_map.get(joint_name, {"q": 0.0, "dq": 0.0})

                        # Compute the target position using the PD-like control
                        #error = cmd["q"] - state["q"]
                        #target_position =  cmd["kp"] * error - cmd["kd"] * state["dq"]
                        #target_position =  state["q"] * 180 / math.pi
                        # Set the target position, stiffness (kp), and damping (kd) for the drive
                        #drive.GetTargetPositionAttr().Set(target_position)
                        drive.GetStiffnessAttr().Set(1.0)
                        drive.GetDampingAttr().Set(0.25)
                        #drive.GetMaxForceAttr().Set(1000000000000.0)
                        #drive.CreateTargetVelocityAttr().Set(0.0)                        

            rclpy.spin_once(ros_node, timeout_sec=0.001)

        simulation_context.stop()
        simulation_app.close()

def main():  
    # Example values
    robot_name = "g1"

    # Camera
    cam_connected_link_name = "torso_link"
    cam_prim_path = f"/{robot_name}/{cam_connected_link_name}/camera"
    robot_position = np.array([2.24816, 0.0, 0.791])
    robot_orientation = np.array([1.0, 0.0, 0.0, 0.0]) # [qw, qx, qy ,qz]
    cam_position = [0.08366, 0.01753, 0.44]
    cam_orientation = [0.40849, 0.0, 0.91276, 0.0] # [qw, qx, qy ,qz]
    scene = Scene(robot_name = robot_name,robot_position=robot_position, robot_orientation = robot_orientation )
    cam_info_topic_ = "/camera/camera/color/camera_info"
    cam_color_topic_ = "/camera/camera/color/image_raw"
    cam_depth_topic_ = "/camera/camera/aligned_depth_to_color/image_raw"
    cam_pcl_topic_ = "/camera/camera/depth/color/points"
    frame_id = "camera_color_optical_frame"
    camera = RealSense(
        cam_prim_path=cam_prim_path, 
        cam_link_name=cam_connected_link_name,
        resolution=(480, 640),
        cam_info_topic = cam_info_topic_,
        cam_color_topic= cam_color_topic_,
        cam_depth_topic= cam_depth_topic_,
        cam_pointcloud_topic= cam_pcl_topic_,
        frame_id = frame_id,
        translation= cam_position,
        orientation = cam_orientation,	
    )

    #  IMU 
    imu_connected_link_name = "pelvis"
    imu_prim_path = f"/{robot_name}/{imu_connected_link_name}/imu_sensor"
    imu_topic = "imu"
    imu_frame_id = "imu_in_pelvis"
    imu=IMUSensor(
    prim_path=imu_prim_path,
    name="imu",
    frequency=200, # or, dt=1./60
    translation=np.array([0, 0, 0]), # or, position=np.array([0, 0, 0]),
    orientation=np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size = 10,
    angular_velocity_filter_size = 10,
    orientation_filter_size = 10,        
    )
    # Create IMU graph 
    config_dir = os.path.dirname(graph_config.__file__)
    imu_graph_name = "imu_graph.yml"
    imu_config_path = os.path.join(config_dir, imu_graph_name)

    imu_graph = Graph(
        config_path=imu_config_path,
        imu_prim_path=imu_prim_path,
        imu_frame_link=imu_frame_id,
        imu_topic=imu_topic,
    )
    og_key_create_node, og_keys_connect, og_keys_set_values = imu_graph.get_graph()
    keys = og.Controller.Keys
    imu_graph_path = f"{imu_prim_path}/IMUActionGraph"

    og.Controller.edit(
        {
            "graph_path": imu_graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: og_key_create_node,
            keys.SET_VALUES: og_keys_set_values,
            keys.CONNECT: og_keys_connect,
        },
    )
    rclpy.init()
    lowcmd_node = LowCmdSubscriber()    
    scene.setup_direct_joint_control() 

    # Spawn Object 
    objects = {
        "ThorlabsTable_modified": { 
            "name": "ThorlabsTable_modified",
            "position": [-1.30311, -0.02765, 1.01311],         
            "orientation": [1.0, 0.0, 0.0, 0.0],         #wxyz
            "scale": [1.0, 1.0, 0.75]
        },
        "Rack_1": {
            "name": "Rack",
            "position": [1.2, 0.5, 0],
            "orientation": [0.70711, 0.0, -0.70711, 0.0],  # wxyz
            "scale": [0.001, 0.001, 0.001]
        },
        "BIW70006056_03": {
            "name": "BIW70006056_03",
            "position": [0.92905, 0.00435, 0.9],
            "orientation": [0.02589, -0.04127, 0.92533, -0.37603],
            "scale": [1.0, 1.0, 1.0]
        }           
    }
    scene.spawn_objects(objects=objects)    
    scene.run(ros_node=lowcmd_node)  

if __name__=="__main__":
    main()