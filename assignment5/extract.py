import time

import pyrealsense2 as rs
import numpy as np
import cv2
from matplotlib import pyplot as plt
import compas.geometry as cg
import compas_rrc as rrc

from ..Assignment4.ARC380_Assignment4 import create_frame_from_points, transform_task_to_world_frame, lift, go_home, go_to_point

from ARC380_Assignment5_helper import capture_img

# COORDINATES
POINT = [132.41, 451.07, 26.12]
POINT_XAXIS = [-89.87, 451.06, 27.10]
POINT_XYPLANE = [29.89, 295.55, 25]

def extract_features(img: np.ndarray) -> list[dict[str, str | float | list[float]]]:
    """Extract features from an image.

    Args:
        img (np.ndarray): The image to extract features from.

    Returns:
        list[dict[str, str | float | np.ndarray]]: A list of dictionaryies--each dictionary belonging to a feature and
            containing relevant characterstic information. Values returned are "color", "shape", "size", "position",
            "orientation".
    """
    pass


if __name__ == '__main__':
    
    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    abb_rrc.send(rrc.SetTool('vac_gripper'))

    abb_speed = 30 # [mm/s]

    task_space_point1 = cg.Point(POINT[0], POINT[1], POINT[2])
    task_space_point2 = cg.Point(POINT_XAXIS[0], POINT_XAXIS[1], POINT_XAXIS[2])
    task_space_point3 = cg.Point(POINT_XYPLANE[0], POINT_XYPLANE[1], POINT_XYPLANE[2])

    ee_point1 = cg.Point(0, 0, 0)
    ee_point2 = cg.Point(1, 0, 0)
    ee_point3 = cg.Point(0, -1, 0)

    task_frame = create_frame_from_points(task_space_point1, task_space_point2, task_space_point3)
    ee_frame = create_frame_from_points(ee_point1, ee_point2, ee_point3)

    origin = transform_task_to_world_frame(ee_frame, task_frame)
