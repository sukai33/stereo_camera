#!/usr/bin/python
# encoding=utf-8

"""
安装UVC相机管理工具 写双目
sudo apt install uvcdynctrl
标定
rosdep install camera_calibration
使用VideoCapture获取双目相机图像
ll /dev/video*
根据需要切换模式：

左单目模式1：LEFT_EYE_MODE
右单目模式2：RIGHT_EYE_MODE
红蓝模式3： RED_BLUE_MODE
双目模式4： BINOCULAR_MODE

将OpenCV图片通CvBridge转成ROS的 sensor_msgs/Image
并将左右双目的图片分别发布到以下话题

左目：/stereo/left/image_raw
右目：/stereo/right/image_raw

将左右两目的相机内参信息, 发布到以下话题

左目：/stereo/left/camera_info    # 左目相机元信息
右目：/stereo/right/camera_info   # 右目相机元信息

rosnode cleanup  清除缓存

"""

#	usb_mode.sh    用这个uvcdynctrl工具给设备/dev/video${id}  发送指令  '(LE)0x50ff'   可以用工具发送也可以用代码发送
"""
#!/bin/bash
id=$1
mode=$2
 
uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x50ff'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x00f6'
uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x2500'
uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x5ffe'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0003'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0002'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0012'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0004'
uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x76c3'
uvcdynctrl -d /dev/video${id} -S 6:10 "(LE)0x0${mode}00"
"""

## 查看相机
#    ll /dev/video*

## 控制台命令切换模式
# rosrun stereo_camera stereo_camera.py
# ./usb_mode.sh 2 1   左单目模式：LEFT_EYE_MODE
# ./usb_mode.sh 2 2   右单目模式：RIGHT_EYE_MODE
# ./usb_mode.sh 2 3   红蓝模式：RED_BLUE_MODE
# ./usb_mode.sh 2 4   双目模式：BINOCULAR_MODE
## 接收topic发送的image图片消息 查看图片
# rqt_image_view

## 148行 选择相机模式:  camera_mode = 'BINOCULAR_MODE'   # 4.双目模式

## 双目相机生成点云
"""
启动驱动
由于为了生成点云相关的数据，我们必须按照stereo_image_proc节点所需要的数据进行发布。这里假设我们以stereo作为ROS命名空间，
则应该向以下四个topic发布数据，
/stereo/left/image_raw      # 左目图像
/stereo/left/camera_info    # 左目相机元信息
/stereo/right/image_raw     # 右目图像
/stereo/right/camera_info   # 右目相机元信息

rqt_image_view
启动立体影像处理
在控制台执行：
$ ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
将stereo_image_proc节点，设置到stereo的命名空间中，其节点会自动订阅相机发布的left和right图像，此方式，
等同于将两个image_proc实例分别运行在stereo/left 和stereo/right命名空间。
此外，此节点会额外输出视差图像（stereo/disparity）和点云图（stereo/points2）
查看单目图像：
$ rosrun image_view image_view image:=/stereo/left/image_rect_color
查看双目图像及视差图像：
$ rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color



"""




"""
##启动项目
rosrun stereo_camera stereo_camera.py 启动项目
rqt_image_view  图像查看
控制台   ./usb_mode.sh 2 4 执行 切换模式
代码中切换模式
148行 选择相机模式:  camera_mode = 'BINOCULAR_MODE'    4.双目模式
启动立体影像处理:
ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
启动rviz
执行rviz或rosrun rviz rviz
控制台运行tf转换就可以显示点云图了
rosrun tf static_transform_publisher 0 0 1 0 0 -1.57 map stereo_image 100

roslaunch一键启动
roslaunch stereo_camera stereo.launch

"""
import cv2
import shlex
import subprocess
import yaml
from os import path

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

        # 准备话题发布者   rqt_image_view
        self.left_img_publisher = rospy.Publisher("stereo/left/image_raw", Image, queue_size=1)
        self.right_img_publisher = rospy.Publisher("stereo/right/image_raw", Image, queue_size=1)

        # 左右相机内参信息发布者   CameraInfo数据类型
        self.left_img_info_pub = rospy.Publisher("stereo/left/camera_info", CameraInfo, queue_size=1)
        self.right_img_info_pub = rospy.Publisher("stereo/right/camera_info", CameraInfo, queue_size=1)
        # opencv图片转data数据之前创建头信息
        self.msg_header = Header()
        # 创建图片转换器
        self.bridge = CvBridge()


        # 当前python文件所在目录
        dir_name = path.dirname(path.realpath(__file__))

		
		#读取左目相机内参文件  通过拼接文的件路径,找到文件left.yaml; path.join:自动判断文件夹dir_name是否需要带/
        self.left_camera_info = self.yaml_to_camera_info(path.join(dir_name, "./calibration/left.yaml"))
		#读取右目相机内参文件
        self.right_camera_info = self.yaml_to_camera_info(path.join(dir_name, "./calibration/right.yaml"))

		
	#cam_id 相机的id 固定是2  cam_mode:模式的id  相机id     ll /dev/video* 查看相机id
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

        # camera_mode = 'LEFT_EYE_MODE'    # 1.左单目模式
        # camera_mode = 'RIGHT_EYE_MODE' # 2.右单目模式
        # camera_mode = 'RED_BLUE_MODE'  # 3.红蓝模式
        camera_mode = 'BINOCULAR_MODE'   # 4.双目模式
        cam_mode = cam_mode_dict[camera_mode]

        self.usb_ctrl(2, cam_mode)

        counter = 0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()

            if cam_mode == 4:  # 双目模式
                # 640 x 480 -> 1280 x 480(1290列 480行)   frame: 原图 None:输出图不需要 fx:水平方向拉伸2倍, fy:竖直方向拉伸1.0 (不变)
                frame_wide = cv2.resize(frame, None, fx=2.0, fy=1.0, interpolation=cv2.INTER_LINEAR)
				#截取左边部分 行,列
                frame_left = frame_wide[0: 480, 0: 640]
				#截取右边部分
                frame_right = frame_wide[0: 480, 640: 1280]
                # cv2.imshow("left", frame_left)
                # cv2.imshow("right", frame_right)
            else:
                frame_left = frame
                frame_right = frame
                # cv2.imshow("frame", frame)
            # opencv图片转data数据之前创建头信息   self.msg_header
             #指定图片id固定的值
            self.msg_header.frame_id = "stereo_image"
			#指定头的时间戳
            self.msg_header.stamp = rospy.Time.now()
            # 发布图片
            self.publish_ros_img(frame_left, self.left_img_publisher)
            self.publish_ros_img(frame_right, self.right_img_publisher)

            # 发布相机信息
            self.left_camera_info.header = self.msg_header
            self.left_img_info_pub.publish(self.left_camera_info)
            self.right_camera_info.header = self.msg_header
            self.right_img_info_pub.publish(self.right_camera_info)

            counter += 1
            if counter % 10 == 0:
                print "publish: {}".format(counter)

            rate.sleep()

        self.capture.release()

    def yaml_to_camera_info(self, param_path):
        """
        读取相机内参文件,转成对象
        :param param_path:  内参文件路径
        :return: CameraInfo
        """
        print "param_path:", param_path

        try:
		   #用with关键字 实现输入输出流自动关闭clos
            with open(param_path, "r") as f:
			    # import yaml
                calib_data = yaml.load(f)
                #CameraInfo 数据类型 ros中的msg
                camera_info = CameraInfo()
				#获取宽高信息
                camera_info.width = calib_data["image_width"]
                camera_info.height = calib_data["image_height"]
				#相机内参
                camera_info.K = calib_data["camera_matrix"]["data"]
				#畸变系数
                camera_info.D = calib_data["distortion_coefficients"]["data"]
				#重映射的一些矩阵
                camera_info.R = calib_data["rectification_matrix"]["data"]
                camera_info.P = calib_data["projection_matrix"]["data"]
				#相机的畸变模型
                camera_info.distortion_model = calib_data["camera_model"]
                return camera_info
        except Exception as e:
            print e

        return None


if __name__ == '__main__':
    camera = StereoCamera()

    camera.run()
