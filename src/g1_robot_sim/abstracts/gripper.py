from typing import List
import os
import numpy as np
from isaacsim import SimulationApp
import omni
from isaacsim.robot.manipulators.grippers import ParallelGripper, SurfaceGripper
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.asset.importer.urdf import _urdf
import omni.graph.core as og
from src.g1_robot_sim.abstracts.graph import Graph
from src.g1_robot_sim import graph_config

# https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.robot.manipulators/docs/index.html#isaacsim.robot.manipulators.grippers.ParallelGripper
class Robotiq2F140(ParallelGripper):
    def __init__(
        self,
        gripper_name: str,
        urdf_path: str,
        end_effector_prim_path: str,
        joint_prim_names: List[str],
        joint_opened_positions: np.ndarray,
        joint_closed_positions: np.ndarray,
        app: SimulationApp,
        *args, **kwargs
    ) -> None:

        self._gripper_name = gripper_name
        self._end_effector_prim_path = end_effector_prim_path
        self._joint_prim_names = joint_prim_names
        self._joint_opened_positions = joint_opened_positions
        self._joint_closed_positions = joint_closed_positions
        self.robot_prim_path = self._import_gripper(urdf_path)
        self._app = app
        super().__init__(
            end_effector_prim_path=self._end_effector_prim_path,
            joint_prim_names = self._joint_prim_names,
            joint_opened_positions = self._joint_opened_positions,
            joint_closed_positions = self._joint_closed_positions,
            *args, **kwargs
        )

    def _import_gripper(self, urdf_path: str) -> str:
        # Set the settings in the import config
        import_config = _urdf.ImportConfig()
        import_config.set_merge_fixed_joints(True)
        import_config.set_convex_decomp(True) # Collision
        import_config.set_fix_base(False)
        import_config.set_make_default_prim(False)
        import_config.set_self_collision(True) #
        import_config.set_create_physics_scene(False)
        import_config.set_import_inertia_tensor(True) # 
        import_config.set_default_drive_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION)

        status, self._gripper_prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            get_articulation_root=True,
        )
        if not status:
            raise RuntimeError("Failed to import URDF file.")

        while is_stage_loading():
            self._app.update()

        self._create_gripper_omnigraph()
        return self._gripper_prim_path

    def _create_gripper_omnigraph(self):
        config_dir = os.path.dirname(graph_config.__file__)
        graph_name = "parallel_gripper_graph.yml"
        config_path_ = os.path.join(config_dir, graph_name)

        gripper_omni_graph = Graph(
            config_path=config_path_,
            gripper_prim_path=self._gripper_prim_path,
        )
        og_key_create_node, og_keys_connect, og_keys_set_values = (
            gripper_omni_graph.get_graph()
        )
        keys = og.Controller.Keys

        graph_path = f"{self._end_effector_prim_path }/{self._gripper_name}/GripperActionGraph"
        (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                keys.CREATE_NODES: og_key_create_node,
                keys.SET_VALUES: og_keys_set_values,
                keys.CONNECT: og_keys_connect,
            },
        )
        return graph_handle

# https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.robot.manipulators/docs/index.html#isaacsim.robot.manipulators.grippers.SurfaceGripper
class RobotiqEpick(SurfaceGripper):
    def __init__(
        self,
        gripper_name: str,
        urdf_path: str,
        end_effector_prim_path: str, 
        app: SimulationApp,
        translate: float = 0, 
        direction: str = 'z', 
        grip_threshold: float = 0.01, 
        force_limit: float = 1000000.0, 
        torque_limit: float = 10000.0, 
        bend_angle: float = 0.1308996938995747, 
        kp: float = 100.0, 
        kd: float = 100.0, 
        disable_gravity: bool = True,
        *args, **kwargs
    ) -> None:
        # Initialize gripper-specific attributes
        self._gripper_name = gripper_name
        self._end_effector_prim_path = end_effector_prim_path
        self._app = app

        # Import gripper model
        self.robot_prim_path = self._import_gripper(urdf_path)
        
        # Initialize parent class (SurfaceGripper) with the appropriate parameters
        super().__init__(
            end_effector_prim_path=end_effector_prim_path,
            translate=translate,
            direction=direction,
            grip_threshold=grip_threshold,
            force_limit=force_limit,
            torque_limit=torque_limit,
            bend_angle=bend_angle,
            kp=kp,
            kd=kd,
            disable_gravity=disable_gravity,
            *args, **kwargs
        )

    def _import_gripper(self, urdf_path: str) -> str:
        # Set the settings in the import config
        import_config = _urdf.ImportConfig()
        import_config.set_merge_fixed_joints(False)
        import_config.set_convex_decomp(True) # Collision
        import_config.set_fix_base(True)
        import_config.set_make_default_prim(False)
        import_config.set_self_collision(True) #
        import_config.set_create_physics_scene(False)
        import_config.set_import_inertia_tensor(False) # 
        import_config.set_default_drive_strength(1047.19751)
        import_config.set_default_position_drive_damping(52.35988)
        import_config.set_default_drive_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION)
        import_config.set_distance_scale(1)
        import_config.set_density(0.0)

        status, self._gripper_prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            get_articulation_root=False,
        )
        if not status:
            raise RuntimeError("Failed to import URDF file.")

        while is_stage_loading():
            self._app.update()

        self._create_gripper_omnigraph()
        return self._gripper_prim_path

    def _create_gripper_omnigraph(self):
        config_dir = os.path.dirname(graph_config.__file__)
        graph_name = "parallel_gripper_graph.yml"
        config_path_ = os.path.join(config_dir, graph_name)

        gripper_omni_graph = Graph(
            config_path=config_path_,
            gripper_prim_path=self._gripper_prim_path,
        )
        og_key_create_node, og_keys_connect, og_keys_set_values = (
            gripper_omni_graph.get_graph()
        )
        keys = og.Controller.Keys

        graph_path = f"{self._end_effector_prim_path}/{self._gripper_name}/GripperActionGraph"
        (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                keys.CREATE_NODES: og_key_create_node,
                keys.SET_VALUES: og_keys_set_values,
                keys.CONNECT: og_keys_connect,
            },
        )
        return graph_handle