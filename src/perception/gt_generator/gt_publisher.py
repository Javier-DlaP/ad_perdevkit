# ROS imports

from ad_perdevkit.msg import GT_3D_Object, GT_3D_Object_list
from carla_msgs.msg import CarlaEgoVehicleInfo

class GT_Object():

    def __init__(self):

        self.type = ""
        self.object_id = 0
        self.alpha = 0
        self.bounding_box_2D_size_x = 0
        self.bounding_box_2D_size_y = 0
        self.bounding_box_2D_center_x = 0
        self.bounding_box_2D_center_y = 0
        self.bounding_box_2D_center_theta = 0
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.h = 0
        self.w = 0
        self.l = 0
        self.rotation_z = 0
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0
        self.truncated = 1
        self.occluded = 3

class GTPublisher():

    def __init__(self, pub):

        self.pub_groundtruth = pub

    def new_msg(self, header):

        self.object_list = GT_3D_Object_list()
        self.object_list.header.stamp = header.stamp
        self.object_list.header.frame_id = ""

    def add_empty_object(self):

        obj = GT_3D_Object()
        self.object_list.gt_3d_object_list.append(obj)

    def add_object(self, gt_object):

        obj = GT_3D_Object()

        obj.type = gt_object.type
        obj.object_id = gt_object.object_id
        obj.alpha = gt_object.alpha

        obj.bounding_box_2D.size_x = gt_object.bounding_box_2D_size_x
        obj.bounding_box_2D.size_y = gt_object.bounding_box_2D_size_y
        obj.bounding_box_2D.center.x = gt_object.bounding_box_2D_center_x
        obj.bounding_box_2D.center.y = gt_object.bounding_box_2D_center_y
        obj.bounding_box_2D.center.theta = gt_object.bounding_box_2D_center_theta

        obj.position.x = gt_object.position_x
        obj.position.y = gt_object.position_y
        obj.position.z = gt_object.position_z

        obj.dimensions.x = gt_object.l
        obj.dimensions.y = gt_object.w
        obj.dimensions.z = gt_object.h

        obj.rotation_z = gt_object.rotation_z

        obj.velocity.x = gt_object.velocity_x
        obj.velocity.y = gt_object.velocity_y
        obj.velocity.z = gt_object.velocity_z

        obj.truncated = gt_object.truncated
        obj.occluded = gt_object.occluded

        self.object_list.gt_3d_object_list.append(obj)

    def publish_groundtruth(self):

        # print(self.object_list)

        self.pub_groundtruth.publish(self.object_list)