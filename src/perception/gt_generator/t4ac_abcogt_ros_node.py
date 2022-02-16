#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# General imports

import signal
import os

# ROS imports

import rospy
from carla_msgs.msg import CarlaEgoVehicleInfo
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from derived_object_msgs.msg import ObjectArray
from ad_perdevkit.msg import GT_3D_Object_list

# Suscribers and Publisher classes

from objects2gt import Objects2GT
from gt_publisher import GTPublisher
from gt2csv import GT2CSV
from store_data import StoreData

class AB4COGT2SORT():

    def create_directories(self):

        csv_path = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/csv_path")
        camera_folder = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/camera_folder")
        lidar_folder = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/lidar_folder")
        radar_folder = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/radar_folder")

        perception_folder = os.path.abspath(os.path.join(csv_path, os.pardir))
        if not os.path.exists(perception_folder):
            os.mkdir(perception_folder)

        for folder in [camera_folder, lidar_folder, radar_folder]:

            if not os.path.exists(folder):
                os.mkdir(folder)

            folder_data = os.path.join(folder, 'data/')

            if not os.path.exists(folder_data):
                os.mkdir(folder_data)

        self.start()

    def start(self):

        node_name = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/node_name")
        rospy.init_node(node_name, anonymous=True)

        camera_info_topic = rospy.get_param("ad_devkit/generate_perception_groundtruth_node/camera_info")
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)

        vehicle_info_topic = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/vehicle_info")
        vehicle_info = rospy.wait_for_message(vehicle_info_topic, CarlaEgoVehicleInfo)

        groundtruth = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/groundtruth")
        pub_groundtruth = rospy.Publisher(groundtruth, GT_3D_Object_list, queue_size=10)

        csv_path = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/csv_path")
        camera_folder = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/camera_folder")
        lidar_folder = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/lidar_folder")
        radar_folder = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/radar_folder")

        store_data = StoreData(camera_folder, lidar_folder, radar_folder)
        gt_publisher = GTPublisher(pub_groundtruth)
        gt_generator = Objects2GT(gt_publisher, camera_info, vehicle_info)
        csv_creator = GT2CSV(csv_path)

        image_color = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/camera")
        radar_points = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/radar")
        point_cloud = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/lidar")
        objects = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/objects")

        rospy.Subscriber(image_color, Image, store_data.store_camera, queue_size=10)
        rospy.Subscriber(radar_points, PointCloud2, store_data.store_radar, queue_size=10)
        rospy.Subscriber(point_cloud, PointCloud2, store_data.store_lidar, queue_size=10)
        
        rospy.Subscriber(point_cloud, PointCloud2, gt_generator.store_pointcloud, queue_size=10)

        rospy.Subscriber(objects, ObjectArray, gt_generator.callback, queue_size=10)

        rospy.Subscriber(groundtruth, GT_3D_Object_list, csv_creator.add2CSV, queue_size=10)

        rospy.spin()

if __name__ == '__main__':

    program = AB4COGT2SORT()

    program.create_directories()
