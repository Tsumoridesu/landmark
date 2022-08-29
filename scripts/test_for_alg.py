# !/usr/bin/env python
# coding = utf-8
from tf import transformations
import yaml
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
# import sys
# import time
# import os
import numpy as np
import math
from geometry_msgs.msg import PoseArray


# from numba import jit, cuda


class landmark_location:
    def __init__(self):
        self.boundingboxes = []
        self.path = "/home/tsumori/catkin_ws/src/landmark_location/landmark_cfg/landmark_list.yaml"
        with open(self.path, "r", encoding="utf-8") as file:
            self.landmark_list = yaml.load(file, Loader=yaml.FullLoader)
        rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, self.cb_image)
        rospy.Subscriber("/particlecloud", PoseArray, self.cb_particle)

    def cb_particle(self, data):
        # for i in data.poses:
        # (r, p, y) = transformations.euler_from_quaternion(
        #     [i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w])
        #     print(y)
        # print("end")
        for i in self.boundingboxes:
            if i.Class == "Vending machine":
                print(i.yaw)
                for j in data.poses:
                    (r, p, y) = transformations.euler_from_quaternion(
                        [j.orientation.x, j.orientation.y, j.orientation.z, j.orientation.w])
                    phi = math.atan2((self.landmark_list['landmark'][0]["Vending machine"][0]['pose'][1] - j.position.y),
                                       self.landmark_list['landmark'][0]["Vending machine"][0]['pose'][0] - j.position.x) + y
                    if phi > math.pi:
                        phi -= 2 * math.pi
                    elif phi < -math.pi:
                        phi += 2 * math.pi

                    theta = phi - i.yaw
                    weight = (math.cos(theta) + 1) * i.probability
                    print(theta, weight)
        # print(self.boundingboxes)
    def cb_image(self, data):
        # for i in data.bounding_boxes:
        #     # cent = (i.xmin + i.xmax) / 2
        #     # yaw_vision = -(cent - 640) / 640 * math.pi
        #     print(i.Class, i.probability, i.yaw)
        # print("end")
        self.boundingboxes = data.bounding_boxes



if __name__ == '__main__':
    rospy.init_node('test')
    landmark_location()
    rospy.spin()
#
# if __name__ == '__main__':
#     tests()

# def linstener():
#     rospy.Subscriber("/particlecloud", PoseArray, cb)
#
#
# def cb(data):
#     for i in data.poses:
#         yaw = math.atan2(2 * (i.orientation.w * i.orientation.z + i.orientation.x * i.orientation.y),
#                          1 - 2 * (i.orientation.y * i.orientation.y + i.orientation.z * i.orientation.z))
#
#         print(yaw)
#     print("end")
#
#
# if __name__ == '__main__':
#     rospy.init_node('test')
#     linstener()
#     rospy.spin()

# l1 = [0, -30]
# o = [0, 0]
# iny = [-(l1[0] - o[0]), (l1[1] - o[1])]
#
# p = math.atan2(iny[0], iny[1])*180/math.pi
#
# print(p)
#
