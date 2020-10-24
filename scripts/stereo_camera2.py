#!/usr/bin/python
# encoding=utf-8

"""
使用VideoCapture获取双目相机图像

根据需要切换模式：

左单目模式1：LEFT_EYE_MODE
右单目模式2：RIGHT_EYE_MODE
红蓝模式3： RED_BLUE_MODE
双目模式4： BINOCULAR_MODE

"""
import cv2
import shlex
import subprocess


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


def usb_ctrl(cam_id, cam_mode):
    for command in command_list:
        command_format = command.format(cam_id=cam_id, cam_mode=cam_mode)
        subprocess.Popen(shlex.split(command_format))


def main():
    capture = cv2.VideoCapture(2)

    if not capture.isOpened():
        print "相机打开失败"
        return

    # camera_mode = 'LEFT_EYE_MODE' # 左单目模式
    # camera_mode = 'RIGHT_EYE_MODE' # 右单目模式
    # camera_mode = 'RED_BLUE_MODE' # 红蓝模式
    camera_mode = 'BINOCULAR_MODE' # 双目模式
    cam_mode = cam_mode_dict[camera_mode]

    usb_ctrl(2, cam_mode)

    while True:
        ret, frame = capture.read()

        if cam_mode == 4: # 双目模式
            # 640 x 480 -> 1280 x 480
            frame_wide = cv2.resize(frame, None, fx=2.0, fy=1.0, interpolation=cv2.INTER_LINEAR)
            frame_left = frame_wide[0: 480, 0: 640]
            frame_right = frame_wide[0: 480, 640: 1280]

            cv2.imshow("left", frame_left)
            cv2.imshow("right", frame_right)

        else:
            cv2.imshow("frame", frame)

        action = cv2.waitKey(30) & 0xff

        if action == 27:
            break

    cv2.destroyAllWindows()
    capture.release()


if __name__ == '__main__':
    main()
