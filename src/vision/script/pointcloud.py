#!/usr/bin/env python

import rospy
import rospkg
import os

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ur5lego.msg import Pose

import numpy as np
from utils.general import cv2
from script.icp import *

# camera to robot
camera_rotation = np.array([0.02216, 0.40428, -0.0211])
camera_translation = np.array([-1.00689257, 0.21422226, -0.34962005])

# robot to table
robot2table_rotation = np.array([[0.0, -1.0, 0.0], 
                                [-1.0, 0.0, 0.0], 
                                [0.0, 0.0, -1.0]])
robot2table_translation = np.array([0.48, 0.43, 0.6])

# simulated camera has a rotation of 
# roll: 2*PI/3
# pitch: 0
# yaw: PI/2
# w.r.t. the world frame
w_R_c = np.matrix([[ 0, -0.49948, 0.86632],
                   [-1.0, 0.0, 0.0],
                   [-0.0, -0.86632, -0.49948]])
x_c = np.array([-0.4, 0.59, 1.4]) #from ur5generics
# x_c = np.array([-0.9, 0.18, -0.35])
base_offset = np.array([0.5, 0.35, 1.75])
camera2table_rotation = np.matrix([[1.0, 0.0, 0.0],
                                   [0.0, -0.5, -0.866],
                                   [0.0, 0.866, -0.5]])
camera2table_translation = np.array([-0.2099953293800354, 0.2773451805114746, 0.5985403060913086])

crop_offset = [672, 399]

class PointCloudHandler:
    def __init__(self, names):
        self.pointcloud_sub = rospy.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2, self.receive_pointcloud)
        self.pointcloud_msg = None
        self.names = names

    def receive_pointcloud(self, msg):
        self.pointcloud_msg = msg
    
    def detect_block(self, boxes, labels):
        while self.pointcloud_msg == None:
            continue

        # pick the center of each bounding box
        centers = [ (int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)) for box in boxes ]

        min_index = 0
        min_distance = float('inf')

        # pick only the nearest point
        for i in range(len(centers)):
            for point in pc2.read_points(self.pointcloud_msg, field_names=['x', 'y', 'z'], uvs=[centers[i]], skip_nans=True):
                distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                if distance < min_distance:
                    min_distance = distance
                    min_index = i

        box = boxes[min_index]
        # decrop box
        box[0] = box[0] + crop_offset[0]
        box[2] = box[2] + crop_offset[0]
        box[1] = box[1] + crop_offset[1]
        box[3] = box[3] + crop_offset[1]

        block_type = labels[min_index] #todo -> map between number and lego piece

        box_pointcloud = []

        for x in range(box[0], box[2]):
            for y in range(box[1], box[3]):
                for point in pc2.read_points(self.pointcloud_msg, field_names=['x', 'y', 'z'], uvs=[(int(x), int(y))], skip_nans=True):
                    box_pointcloud.append(np.array(point))
        
        box_pointcloud = self.filter_pointcloud(np.array(box_pointcloud), 0.05)
        
        source, target = self.get_pointclouds(box_pointcloud=box_pointcloud, block_type=block_type)

        # find transformation of the block in the box
        transformation = self.find_transformation(source, target)
        
        print(transformation)
        translation = transformation[:3, 3]

        # Calculate pitch, yaw, and roll angles using trigonometric functions
        pitch = np.arcsin(-transformation[1, 2])
        roll = np.arctan2(transformation[1, 0] / np.cos(pitch), transformation[1, 1] / np.cos(pitch))
        yaw = np.arctan2(transformation[0, 3] / np.cos(pitch), transformation[2, 2] / np.cos(pitch))
        
        response = Pose()
        response.position.x = translation[0]
        response.position.y = translation[1]
        response.position.z = translation[2]
        response.orientation.x = roll
        response.orientation.y = pitch
        response.orientation.z = yaw
        response.legoType = block_type
        
        return response

    def get_pointclouds(self, box_pointcloud, block_type):
        # load pointclouds
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(box_pointcloud)
        target.transform(self.camera_to_table())
        
        mesh = o3d.io.read_triangle_mesh(self.get_stl_path(block_type))
        source = mesh.sample_points_uniformly(number_of_points=10000)

        rotation = np.matrix([[0.0, -1.0, 0.0, 0.0], 
                              [1.0, 0.0, 0.0, 0.0], 
                              [0.0, 0.0, 1, 0],
                              [0.0, 0.0, 0.0, 1.0]])
                            
        source.transform(rotation)
        
        # draw_registration_result(source=source, target=target, transformation=np.identity(4))
        return source, target

    def find_transformation(self, source, target):
        # downsample the pointclouds and compute fpfh features
        source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, 0.001)

        # match fpfh features using ransac algorithm for global registration
        transf_guess = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh)
        # draw_registration_result(source_down, target_down, transf_guess.transformation)

        # refine the result using icp, starting from the ransac result
        result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh, transf_guess.transformation)
        
        # draw_registration_result(source, target, result_icp.transformation)
        
        return result_icp.transformation

    def filter_pointcloud(self, pointcloud, threshold=0.005):
        # Find the maximum z-index
        max_z = np.max(pointcloud[:, 2])  

        # Create a boolean mask based on the condition
        mask = pointcloud[:, 2] < max_z - threshold

        # Use boolean indexing to keep only the points that satisfy the condition
        filtered_points = pointcloud[mask]

        return filtered_points

    def camera_to_table(self):
        table2camera = camera2table_rotation.T.dot(-camera2table_translation)

        # camera frame to table frame
        return self.get_transf_matrix(camera2table_rotation.T, table2camera)
        
    def get_transf_matrix(self, rotation_matrix, translation_vector):
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation_vector
        return transformation_matrix
    
    def get_stl_path(self, block_type):
        block_name = self.names[block_type]
        
        # Create an instance of the RosPack class
        rospack = rospkg.RosPack()
        # Check if a specific package exists
        package_name = "ur5lego"
        package_path = rospack.get_path(package_name)
        
        stl_path = os.path.join(package_path, 'blocks_description/stl', block_name + '.stl')
        
        return stl_path
