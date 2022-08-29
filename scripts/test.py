# coding:utf-8

import roslib;
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

path = '/home/tsumori/workspace/Data/tsudanuma/'  # 存放图片的位置


class ImageCreator():

    def __init__(self):
        self.bridge = CvBridge()
        i = 0
        j = 0
        with rosbag.Bag('/media/tsumori/DATA/2022-08-15-15-48-39.bag', 'r') as bag:  # 要读取的bag文件；
            for topic, msg, t in bag.read_messages():
                if topic == "/usb_cam/image_raw":  # 图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    name_format = "tsudanuma_%s_000019_leftImg8bit" % "{:06d}".format(j)
                    # image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    image_name = name_format + ".png"  # 图像命名：时间戳.jpg
                    i += 1
                    if i >= 10:
                        cv_image = cv2.resize(cv_image, (2048, 1024), interpolation=cv2.INTER_CUBIC)
                        cv2.imwrite(path + image_name, cv_image)  # 保存；
                        j += 1
                        i = 0


if __name__ == '__main__':

    # rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
