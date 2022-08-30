#!/usr/bin/env python3
# coding = utf-8
from tf import transformations
import yaml
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
import numpy as np
import math
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray


class landmark_location:
    def __init__(self):

        self.boundingboxes = []
        self.path = "/home/tsumori/catkin_ws/src/landmark_location/landmark_cfg/landmark_list.yaml"
        with open(self.path, "r", encoding="utf-8") as file:
            self.landmark_list = yaml.load(file, Loader=yaml.FullLoader)
        rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, self.cb_image)
        rospy.Subscriber("/particlecloud", PoseArray, self.cb_particle)
        self.pub = rospy.Publisher("/vision_weight", Float64MultiArray, queue_size=1)
        self.vision_weight = []
        self.vision_weight_pub = Float64MultiArray()

    def cb_particle(self, data):
        for i in self.boundingboxes:
            if i.Class == "Vending machine":
                for j in data.poses:
                    (r, p, y) = transformations.euler_from_quaternion(
                        [j.orientation.x, j.orientation.y, j.orientation.z, j.orientation.w])
                    phi = math.atan2(
                        (self.landmark_list['landmark'][0]["Vending machine"][0]['pose'][1] - j.position.y),
                        self.landmark_list['landmark'][0]["Vending machine"][0]['pose'][0] - j.position.x) + y
                    if phi > math.pi:
                        phi -= 2 * math.pi
                    elif phi < -math.pi:
                        phi += 2 * math.pi

                    theta = phi - i.yaw
                    weight = (math.cos(theta) + 1) * i.probability
                    self.vision_weight.append(weight)
            else:
                self.vision_weight_pub = Float64MultiArray()
        self.vision_weight_pub = Float64MultiArray(data=self.vision_weight)
        self.pub.publish(self.vision_weight_pub)

    def cb_image(self, data):
        self.boundingboxes = data.bounding_boxes


if __name__ == '__main__':
    rospy.init_node('landmark_localization')
    landmark_location()
    rospy.spin()
