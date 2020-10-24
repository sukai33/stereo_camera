#!/usr/bin/python
# encoding=utf-8

"""
使用VideoCapture获取双目相机图像

根据需要切换模式：

左单目模式1：LEFT_EYE_MODE
右单目模式2：RIGHT_EYE_MODE
红蓝模式3： RED_BLUE_MODE
双目模式4： BINOCULAR_MODE

将OpenCV图片通CvBridge转成ROS的 sensor_msgs/Image
并将左右双目的图片分别发布到以下话题

左目：stereo/left/image_raw
右目：stereo/right/image_raw
"""
import cv2
import shlex
import subprocess

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import rospy

cam_mode_dict = {
    'LEFT_EYE_MODE': 1,
    'RIGHT_EYE_MODE': 2,
    'RED_BLUE_MODE': 3,
    'BINOCULAR_MODE': 4,
}

command_list = [
    "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x50ff'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x50f6'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x2500'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x5ffe'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0003'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0002'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0012'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:15 '(LE)0x0004'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:8 '(LE)0x76c3'",
    "uvcdynctrl -d /dev/video{cam_id} -S 6:10 '(LE)0x0{cam_mode}00'",
]


class StereoCamera:

    def __init__(self):

        self.capture = cv2.VideoCapture(2)

        # 初始化节点信息
        rospy.init_node("stereo_camera_node", anonymous=True)

        # 准备话题发布者
        self.left_img_publisher = rospy.Publisher("stereo/left/image_raw", Image, queue_size=1)
        self.right_img_publisher = rospy.Publisher("stereo/right/image_raw", Image, queue_size=1)

        self.msg_header = Header()
        # 创建图片转换器
        self.bridge = CvBridge()

    def usb_ctrl(self, cam_id, cam_mode):
        for command in command_list:
            command_format = command.format(cam_id=cam_id, cam_mode=cam_mode)
            subprocess.Popen(shlex.split(command_format))

    def publish_ros_img(self, frame, img_publisher):
        # 将cv图片转成ros的Image
        try:
            ros_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")

            ros_img.header = self.msg_header
            # 将其发布到指定话题
            img_publisher.publish(ros_img)

        except CvBridgeError as e:
            print e

    def run(self):

        if not self.capture.isOpened():
            print "相机打开失败"
            return

        # camera_mode = 'LEFT_EYE_MODE' # 左单目模式
        # camera_mode = 'RIGHT_EYE_MODE' # 右单目模式
        # camera_mode = 'RED_BLUE_MODE' # 红蓝模式
        camera_mode = 'BINOCULAR_MODE'  # 双目模式
        cam_mode = cam_mode_dict[camera_mode]

        self.usb_ctrl(2, cam_mode)

        counter = 0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()

            if cam_mode == 4:  # 双目模式
                # 640 x 480 -> 1280 x 480
                frame_wide = cv2.resize(frame, None, fx=2.0, fy=1.0, interpolation=cv2.INTER_LINEAR)
                frame_left = frame_wide[0: 480, 0: 640]
                frame_right = frame_wide[0: 480, 640: 1280]
                # cv2.imshow("left", frame_left)
                # cv2.imshow("right", frame_right)
            else:
                frame_left = frame
                frame_right = frame
                # cv2.imshow("frame", frame)


            self.msg_header.frame_id = "stereo_image"
            self.msg_header.stamp = rospy.Time.now()

            self.publish_ros_img(frame_left, self.left_img_publisher)
            self.publish_ros_img(frame_right, self.right_img_publisher)

            counter += 1
            if counter % 10 == 0:
                print "publish: {}".format(counter)

            rate.sleep()

        self.capture.release()


if __name__ == '__main__':
    camera = StereoCamera()

    camera.run()
