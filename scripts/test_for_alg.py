#!/usr/bin/env python3
# coding=utf-8
import sys

sys.path.append('/home/tsumori/catkin_ws/src/yolov5_pytorch_ros/src')
from tf import transformations
import yaml
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
import math
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray


class landmark_location:
    def __init__(self):

        self.boundingboxes = []

        # landmark_list の読み込み
        self.path = "/home/tsumori/catkin_ws/src/landmark_location/landmark_cfg/landmark_list.yaml"
        with open(self.path, "r", encoding="utf-8") as file:
            self.landmark_list = yaml.load(file, Loader=yaml.FullLoader)

        # subscribe topic の設定
        rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, self.cb_image)
        rospy.Subscriber("/particlecloud", PoseArray, self.cb_particle)

        # publish topic の設定
        self.pub = rospy.Publisher("/vision_weight", Float64MultiArray, queue_size=1)

    def cb_particle(self, data):
        vision_weight = []
        for i in self.boundingboxes:
            # Vending machine　のみを対象とする 完成の場合は i.Class は landmark_list にあるかどうかを確認し、
            # あるので場合のみi.classをlandmark_listのkeyにする
            if i.Class == "Vending machine":
                for j in data.poses:
                    (r, p, y) = transformations.euler_from_quaternion(
                        [j.orientation.x, j.orientation.y, j.orientation.z, j.orientation.w])

                    # 追加予定、距離でランドマークを選べる

                    phi = math.atan2(
                        (self.landmark_list['landmark'][0]["Vending machine"][0]['pose'][1] - j.position.y),
                        self.landmark_list['landmark'][0]["Vending machine"][0]['pose'][0] - j.position.x) + y
                    if phi > math.pi:
                        phi -= 2 * math.pi
                    elif phi < -math.pi:
                        phi += 2 * math.pi

                    theta = phi - i.yaw
                    weight = (math.cos(theta) + 1) * i.probability
                    vision_weight.append(weight)
        vision_weight_pub = Float64MultiArray(data=vision_weight)
        self.pub.publish(vision_weight_pub)

    def cb_image(self, data):
        self.boundingboxes = data.bounding_boxes


if __name__ == '__main__':
    rospy.init_node('landmark_localization')
    landmark_location()
    rospy.spin()
