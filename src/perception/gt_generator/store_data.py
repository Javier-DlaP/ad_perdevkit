# General imports

from PIL import Image
import os
import glob
import time
import cv2
import numpy as np

# ROS imports

import rospy
import sensor_msgs

class StoreData():

    def __init__(self, camera_folder, lidar_folder, radar_folder):

        self.camera_folder = camera_folder
        self.lidar_folder = lidar_folder
        self.radar_folder = radar_folder

        self.camera_id = 0
        self.lidar_id = 0
        self.radar_id = 0

        # Flag to don't save more than one pointcloud with the same timestamp
        self.previous_nsecs_lidar = 0

        for folder_sensor in [self.camera_folder, self.lidar_folder, self.radar_folder]:
            files_data = glob.glob(os.path.join(folder_sensor, "data/*"))

            for f in files_data:
                os.remove(f)
            files = glob.glob(os.path.join(folder_sensor, "*"))

            for f in files:
                if os.path.isfile(f):
                    os.remove(f)

        info_camera = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/camera_info")
        msg_info_camera = rospy.wait_for_message(info_camera, sensor_msgs.msg.CameraInfo)

        lidar_topic = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/lidar")
        msg_lidar = rospy.wait_for_message(lidar_topic, sensor_msgs.msg.PointCloud2)

        radar_topic = rospy.get_param("/ad_devkit/generate_perception_groundtruth_node/radar")
        msg_radar = rospy.wait_for_message(radar_topic, sensor_msgs.msg.PointCloud2)

        self.write_matrix(msg_info_camera, self.camera_folder)
        self.readme_pointcloud2(msg_lidar, self.lidar_folder)
        self.readme_pointcloud2(msg_radar, self.radar_folder)

    def store_camera(self, camera_msg):

        image = Image.frombytes("RGBA", (camera_msg.width, camera_msg.height), camera_msg.data)
        image = image.convert('RGB')
        open_cv_image = np.array(image)

        path_data = path = os.path.join(self.camera_folder, "data")
        path = os.path.join(path_data, str(self.camera_id).zfill(10) + ".png")

        cv2.imwrite(path, open_cv_image)

        self.timestamps(camera_msg, self.camera_folder)
        
        self.camera_id += 1

    def store_lidar(self, lidar_msg):

        self.previous_nsecs_lidar = lidar_msg.header.stamp.nsecs

        path_data = path = os.path.join(self.lidar_folder, "data")

        lidar_bin = open(os.path.join(path_data, str(self.lidar_id).zfill(10) + ".bin"), 'ab')
        lidar_bin.write(lidar_msg.data)
        lidar_bin.close()

        self.timestamps(lidar_msg, self.lidar_folder)

        self.lidar_id += 1

    def store_radar(self, radar_msg):

        path_data = path = os.path.join(self.radar_folder, "data")
        radar_bin = open(os.path.join(path_data, str(self.radar_id).zfill(10) + ".bin"), 'ab')

        radar_bin.write(radar_msg.data)
        radar_bin.close()

        self.timestamps(radar_msg, self.radar_folder)

        self.radar_id += 1

    def timestamps(self, msg, folder):

        timestamps = open(os.path.join(folder, "timestamp.txt"), 'a')
        timestamps.write(str(msg.header.stamp.secs + float(msg.header.stamp.nsecs)/1000000000) + "\n")
        timestamps.close()

    def write_matrix(self, msg, folder):

        matrix_file = open(os.path.join(folder, "intrinsic_matrix.txt"), 'a')
        matrix = list(msg.P)
        for i in range(len(matrix)):
            matrix_file.write(str(matrix[i]))
            if i+1 != len(matrix):
                matrix_file.write(",")
        matrix_file.close()

    def readme_pointcloud2(self, msg, folder):

        readme = open(os.path.join(folder, "readme.txt"), 'a')
        readme.write("fields:\n"+str(msg.fields)+"\n\n")
        readme.write("is_bigendian:\n"+str(msg.is_bigendian)+"\n\n")
        readme.write("point_step:\n"+str(msg.point_step)+"\n\n")
        readme.close()