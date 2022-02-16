import time

# ROS imports

from ad_perdevkit.msg import GT_3D_Object_list, GT_3D_Object

class GT2CSV():

    def __init__(self, path):

        self.groundtruth_lists = []
        self.path_csv = path

        # Create the CSV and remove the previous if there's already one
        csv = open(self.path_csv, 'w')
        csv.write("frame,timestamp,id,type,alpha,left,top,right,bottom,l,w,h,x,y,z,rotation_z,vx,vy,vz,truncated,occluded\n")
        csv.close()

    def add2CSV(self, groundtruth):
        
        groundtruth_list_header = []
        groundtruth_list_header.append(groundtruth.header.seq) # frame
        groundtruth_list_header.append(groundtruth.header.stamp.secs + float(groundtruth.header.stamp.nsecs)/1000000000) # timestamp

        for gt_object in groundtruth.gt_3d_object_list:

            if gt_object != GT_3D_Object():
            
                groundtruth_list = list(groundtruth_list_header)
                groundtruth_list.append(gt_object.object_id) # id
                groundtruth_list.append(gt_object.type) # type
                groundtruth_list.append(gt_object.alpha) # alpha
                groundtruth_list.append(int(gt_object.bounding_box_2D.center.x - gt_object.bounding_box_2D.size_x / 2)) # left
                groundtruth_list.append(int(gt_object.bounding_box_2D.center.y - gt_object.bounding_box_2D.size_y / 2)) # top
                groundtruth_list.append(int(gt_object.bounding_box_2D.center.x + gt_object.bounding_box_2D.size_x / 2)) # right
                groundtruth_list.append(int(gt_object.bounding_box_2D.center.y + gt_object.bounding_box_2D.size_y / 2)) # bottom
                groundtruth_list.append(gt_object.dimensions.x) # l
                groundtruth_list.append(gt_object.dimensions.y) # w
                groundtruth_list.append(gt_object.dimensions.z) # h
                groundtruth_list.append(gt_object.position.x) # x
                groundtruth_list.append(gt_object.position.y) # y
                groundtruth_list.append(gt_object.position.z) # z
                groundtruth_list.append(gt_object.rotation_z) # rotation_z
                groundtruth_list.append(gt_object.velocity.x) # vx
                groundtruth_list.append(gt_object.velocity.y) # vy
                groundtruth_list.append(gt_object.velocity.z) # vz
                groundtruth_list.append(gt_object.truncated) # truncated
                groundtruth_list.append(gt_object.occluded) # occluded

                self.groundtruth_lists.append(groundtruth_list)

        self.saveCSV()

    def saveCSV(self):

        csv = open(self.path_csv, 'a')

        for groundtruth_list in self.groundtruth_lists:

            len_groundtruth_list = len(groundtruth_list)
            for i in range(len_groundtruth_list):
                csv.write(str(groundtruth_list[i]))

                if(i+1 != len_groundtruth_list):
                    csv.write(",")

            csv.write("\n")

        csv.close()

        self.groundtruth_lists = []