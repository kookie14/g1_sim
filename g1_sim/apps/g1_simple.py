# Copyright (c) 2025 VinRobotics. All rights reserved

import os

from isaacsim import SimulationApp

CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)
"""Rest everything follows."""

# More imports that need to compare after we create the app
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    stage,
)

# extensions.enable_extension("omni.isaac.ros2_bridge")

from g1_robot_sim import assets


EVN_PATH = os.path.dirname(assets.__file__) + "/env/factory/Collected_warehouse_with_forklifts/warehouse.usd"
BACKGROUND_STAGE_PATH = "/World/Warehouse"
stage.add_reference_to_stage(EVN_PATH, BACKGROUND_STAGE_PATH)



### TAKE A LONG TIME TO LOAD LARGE SCENE
# EVN_PATH = "../Collected_warehouse/warehouse.usd"
# BACKGROUND_STAGE_PATH = "/SM_floor53"
# stage.add_reference_to_stage(EVN_PATH, BACKGROUND_STAGE_PATH)
###
from scene import Scene, create_rgbd_camera, create_imu




def main():
    # Example values
    robot_name = "g1_29dof_rev_1_0"
    robot_file_name = "g1"
    target_prim_joint_name = "left_hip_pitch_joint"

    # Scene
    scene = Scene(
        sim_app=simulation_app,
        robot_name=robot_name,
        robot_file_name=robot_file_name,
        target_prim_joint_name=target_prim_joint_name,
    )

    # Robot
    scene.spawn_robot(
        robot_position=[0.0, 0.0, 0.79311],
        robot_orientation=[1.0, 0.0, 0.0, 0.0],  # [qw, qx, qy ,qz]
    )

    # #HEAD CAM 
    # #TODO replace fx fy cx cy by your camera intrinsic 
    # head_fx = 4.4260431700991404e+02
    # head_fy = 4.6152026668010120e+02
    # head_cx = 6.3584834258781643e+02
    # head_cy = 3.8708963117313408e+02
    # create_rgbd_camera(
    #     cam_prim_path=cam_prim_path,
    #     cam_link_name=upper_body_root_link_name,
    #     cam_position=[0.08366, 0.01753, 0.44],
    #     cam_orientation=[0.40849, 0.0, 0.91276, 0.0],
    #     cam_resolution=(640, 480),
    #     cam_dimension=(640, 480),
    #     cam_focal_length=(head_fx, head_fy),
    #     cam_optical_center=(head_cx, head_cy),
    # )

    # # IMU
    # create_imu(
    #     imu_prim_path=imu_prim_path,
    #     imu_link_name=lower_body_root_link_name,
    # )

    # objects = {
    #     "ThorlabsTable": {
    #         "name": "ThorlabsTable",
    #         "position": [1.2, -0.02765, 0.81912],
    #         "orientation": [0.0, 0.0, 0.0, -1.0],  
    #         "scale": [1.0, 1.0, 1.0],
    #     },
    #     "mustard": {
    #         "name": "mustard",
    #         "position": [0.6141, 0.09126, 0.89935],
    #         "orientation": [0.0, 0.0, 0.0, -1.0],
    #         "scale": [1.0, 1.0, 1.0],
    #     },
    #     "cracker_box": {
    #         "name": "cracker_box",
    #         "position": [0.77648, 0.18337, 0.90998],
    #         "orientation": [0.0, 0.0, 0.0, -1.0],
    #         "scale": [1.0, 1.0, 1.0],
    #     },
    #     "chewing_gum_cool_air": {
    #         "name": "chewing_gum_cool_air",
    #         "position": [0.64641, -0.09741, 0.84204],
    #         "orientation": [0.0, 0.0, 0.0, -1.0],
    #         "scale": [1.0, 1.0, 1.0],
    #     },
    # }
    # scene.spawn_objects(objects=objects)

    scene.run()


if __name__ == "__main__":
    main()
