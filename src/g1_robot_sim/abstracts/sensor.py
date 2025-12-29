# Copyright (c) 2025 VinRobotics. All rights reserved
# Reference:
#   https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_camera.html
#   https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_physics_imu.html

import os
from typing import Optional

import numpy as np
import omni.graph.core as og
from isaacsim.sensors.camera import Camera
from isaacsim.sensors.physics import IMUSensor

from src.g1_robot_sim import graph_config
from src.g1_robot_sim.abstracts.graph import Graph


class BaseCamera(Camera):
    def __init__(
        self,
        cam_prim_path: str,
        cam_link_name: str,
        cam_resolution: tuple[int, int],  # width, height
        cam_dimension: tuple[int, int],  # width, height
        cam_focal_length: tuple[float, float],  # fx, fy
        cam_optical_center: tuple[float, float],  # cx, cy
        cam_info_topic: str,
        cam_color_topic: str,
        cam_depth_topic: str = None,
        cam_pointcloud_topic: str = None,
        cam_frequency: int = 30,
        cam_projection_types: str = "pinhole",
        cam_frame_id: str = "camera_color_optical_frame",
        cam_graph_name: str = "camera_rgb_graph.yml",
        cam_translation: Optional[np.ndarray] = None,  # [x, y, z]
        cam_orientation: Optional[np.ndarray] = None,  # [qw, qx, qy ,qz]
        diagonal_fov: int = None,
        distortion_coefficients: tuple[float, float, float, float] = None,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(
            prim_path=cam_prim_path,
            frequency=cam_frequency,
            resolution=cam_resolution,
            *args,
            **kwargs,
        )

        self._cam_prim_path = cam_prim_path
        self._cam_link_name = cam_link_name
        self._width = cam_dimension[0]
        self._height = cam_dimension[1]
        self.horizontal_apeture = cam_resolution[0]
        self.vertical_aperture = cam_resolution[1]
        self._fx = cam_focal_length[0]
        self._fy = cam_focal_length[1]
        self._cx = cam_optical_center[0]
        self._cy = cam_optical_center[1]
        self._cam_info_topic = cam_info_topic
        self._cam_color_topic = cam_color_topic
        self._cam_depth_topic = cam_depth_topic
        self._cam_pointcloud_topic = cam_pointcloud_topic
        self._camera_frame_id = cam_frame_id
        self._cam_graph_name = cam_graph_name

        self.initialize()
        self.set_resolution((cam_resolution[0], cam_resolution[1]))
        self.set_local_pose(
            translation=cam_translation, orientation=cam_orientation, camera_axes="ros"
        )
        self.set_projection_type(cam_projection_types)

        # Pixel size in microns, aperture and focus distance from the camera sensor specification
        # Note: to disable the depth of field effect, set the f_stop to 0.0. This is useful for debugging.
        pixel_size = 3 * 1e-3   # pixel size, in mm.
        f_stop = 1.8            # f-number, the ratio of the lens focal length to the diameter of the entrance pupil
        focus_distance = 1.5    # The distance from the camera to the object plane in meters.

        # Calculate the focal length and aperture size from the camera matrix
        # ((fx, _, cx),
        #  (_, fy, cy),
        #  (_, _, _))
        horizontal_aperture = pixel_size * self.horizontal_apeture  # The aperture size in mm
        vertical_aperture = pixel_size * self.vertical_aperture
        focal_length_x = self._fx * pixel_size
        focal_length_y = self._fy * pixel_size
        focal_length = (focal_length_x + focal_length_y) / 2  # The focal length in mm

        # Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
        # Convert from mm to cm (or 1/10th of a world unit)
        self.set_focal_length(focal_length / 10.0)
        self.set_focus_distance(focus_distance)  # The focus distance in meters
        self.set_lens_aperture(f_stop * 100.0)  # Convert the f-stop to Isaac Sim units
        # Convert from mm to cm (or 1/10th of a world unit)
        self.set_horizontal_aperture(horizontal_aperture / 10.0)
        self.set_vertical_aperture(vertical_aperture / 10.0)

        self.set_clipping_range(0.05, 1.0e5)

        if cam_projection_types == "fisheyeKannalaBrandtK3":
            self.set_kannala_brandt_properties(
                nominal_width=self._width,
                nominal_height=self._height,
                optical_centre_x=self._cx,
                optical_centre_y=self._cy,
                max_fov=diagonal_fov,
                distortion_model=distortion_coefficients,
            )
        elif cam_projection_types == "fisheyePolynomial":
            self.set_matching_fisheye_polynomial_properties(
                nominal_width=self._width,
                nominal_height=self._height,
                optical_centre_x=self._cx,
                optical_centre_y=self._cy,
                max_fov=diagonal_fov,
                distortion_model=distortion_coefficients,
            )
        else:
            # No fisheye
            pass

        # Omni Action Graph
        self._create_omnigraph()

    def _create_omnigraph(self):
        config_dir = os.path.dirname(graph_config.__file__)
        config_path_ = os.path.join(config_dir, self._cam_graph_name)
        if self._cam_depth_topic and self._cam_pointcloud_topic:
            graph = Graph(
                config_path=config_path_,
                prim_path=self._cam_prim_path,
                height=self._height,
                width=self._width,
                cam_info_topic=self._cam_info_topic,
                cam_color_topic=self._cam_color_topic,
                cam_depth_topic=self._cam_depth_topic,
                cam_pointcloud_topic=self._cam_pointcloud_topic,
                frame_id=self._camera_frame_id,
            )
        else:
            graph = Graph(
                config_path=config_path_,
                prim_path=self._cam_prim_path,
                height=self._height,
                width=self._width,
                cam_info_topic=self._cam_info_topic,
                cam_color_topic=self._cam_color_topic,
                frame_id=self._camera_frame_id,
            )
        og_key_create_node, og_keys_connect, og_keys_set_values = graph.get_graph()
        keys = og.Controller.Keys
        # TODO: Fix this: get attribute path, and pass it here
        graph_path = f"{self._cam_prim_path}/CameraActionGraph"  # /{cam_link_name}
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


class RealSense(BaseCamera):
    def __init__(
        self,
        cam_prim_path: str,
        cam_link_name: str,
        cam_resolution: tuple[int, int],  # width, height
        cam_dimension: tuple[int, int],  # width, height
        cam_focal_length: tuple[float, float],  # fx, fy
        cam_optical_center: tuple[float, float],  # cx, cy
        cam_info_topic: str,
        cam_color_topic: str,
        cam_depth_topic: str,
        cam_pointcloud_topic: str,
        cam_frequency: int = 30,
        cam_projection_types: str = "pinhole",
        cam_frame_id: str = "camera_color_optical_frame",
        cam_graph_name: str = "camera_rgbd_graph.yml",
        cam_translation: Optional[np.ndarray] = None,
        cam_orientation: Optional[np.ndarray] = None,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(
            cam_prim_path=cam_prim_path,
            cam_link_name=cam_link_name,
            cam_resolution=cam_resolution,
            cam_dimension=cam_dimension,
            cam_optical_center=cam_optical_center,
            cam_focal_length=cam_focal_length,
            cam_info_topic=cam_info_topic,
            cam_color_topic=cam_color_topic,
            cam_depth_topic=cam_depth_topic,
            cam_pointcloud_topic=cam_pointcloud_topic,
            cam_frequency=cam_frequency,
            cam_projection_types=cam_projection_types,
            cam_frame_id=cam_frame_id,
            cam_graph_name=cam_graph_name,
            cam_translation=cam_translation,
            cam_orientation=cam_orientation,
            diagonal_fov=None,
            distortion_coefficients=None,
            *args,
            **kwargs,
        )


class KannalaBrandt(BaseCamera):
    def __init__(
        self,
        cam_prim_path: str,
        cam_link_name: str,
        cam_resolution: tuple[int, int],  # width, height
        cam_dimension: tuple[int, int],  # width, height
        cam_focal_length: tuple[float, float],  # fx, fy
        cam_optical_center: tuple[float, float],  # cx, cy
        cam_info_topic: str,
        cam_color_topic: str,
        cam_frequency: int = 30,
        cam_projection_types: str = "fisheyeKannalaBrandtK3",
        cam_frame_id: str = "camera_color_optical_frame",
        cam_graph_name: str = "camera_rgb_graph.yml",
        cam_translation: Optional[np.ndarray] = None,
        cam_orientation: Optional[np.ndarray] = None,
        distortion_coefficients: tuple[float, float, float, float] = [0.05, 0.01, -0.003, -0.0005],
        *args,
        **kwargs,
    ) -> None:
        super().__init__(
            cam_prim_path=cam_prim_path,
            cam_link_name=cam_link_name,
            cam_resolution=cam_resolution,
            cam_dimension=cam_dimension,
            cam_optical_center=cam_optical_center,
            cam_focal_length=cam_focal_length,
            cam_info_topic=cam_info_topic,
            cam_color_topic=cam_color_topic,
            cam_depth_topic=None,
            cam_pointcloud_topic=None,
            cam_frequency=cam_frequency,
            cam_projection_types=cam_projection_types,
            cam_frame_id=cam_frame_id,
            cam_graph_name=cam_graph_name,
            cam_translation=cam_translation,
            cam_orientation=cam_orientation,
            diagonal_fov=None,
            distortion_coefficients=distortion_coefficients,
            *args,
            **kwargs,
        )


class IMU(IMUSensor):
    def __init__(
        self,
        imu_prim_path: str,
        imu_link_name: str,
        imu_topic: str,
        imu_frame_id: Optional[str] = "imu_sensor",
        imu_frequency: Optional[int] = 60,
        imu_translation: Optional[np.ndarray] = None,
        imu_orientation: Optional[np.ndarray] = None,
        imu_linear_acceleration_filter_size: Optional[int] = 10,
        imu_angular_velocity_filter_size: Optional[int] = 10,
        imu_orientation_filter_size: Optional[int] = 10,
        *args,
        **kwargs,
    ) -> None:
        self._imu_prim_path = imu_prim_path
        self._imu_link_name = imu_link_name
        self._imu_topic = imu_topic
        self._imu_frame_id = imu_frame_id

        super().__init__(
            prim_path=self._imu_prim_path,
            frequency=imu_frequency,  # or, dt=1./60
            translation=imu_translation,
            orientation=imu_orientation,
            linear_acceleration_filter_size=imu_linear_acceleration_filter_size,
            angular_velocity_filter_size=imu_angular_velocity_filter_size,
            orientation_filter_size=imu_orientation_filter_size,
            *args,
            **kwargs,
        )

        # Omni Action Graph
        self._create_imu_omnigraph()

    def _create_imu_omnigraph(self):
        config_dir = os.path.dirname(graph_config.__file__)
        graph_name = "imu_graph.yml"
        config_path_ = os.path.join(config_dir, graph_name)
        graph = Graph(
            config_path=config_path_,
            imu_prim_path=self._imu_prim_path,
            imu_topic=self._imu_topic,
            imu_frame_id=self._imu_frame_id,
        )
        og_key_create_node, og_keys_connect, og_keys_set_values = graph.get_graph()
        keys = og.Controller.Keys

        # TODO: Fix this: get attribute path, and pass it here
        graph_path = f"{self._imu_prim_path}/IMUActionGraph"
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
