# -*-coding:utf-8-*-

import time
import cv2
import yaml
import global_var
from robomaster import robot
from image_detect import ImageDetect

POS_LOW = -20
POS_MID = 0
POS_HIGH = 20
POS_TOP = 220
ARM_POS = POS_MID


# =============================================================================
# 一般功能测试
# -----------------------------------------------------------------------------
def test_func():
    img = cv2.imread("./images/t1.jpg")
    # detect_tennis(img)
    cv2.imshow("roboep_tennis", img)
    cv2.waitKey(10000)
    cv2.destroyAllWindows()


# =============================================================================
# 距离传感器数据处理的回调函数
# -----------------------------------------------------------------------------
def sub_data_handler(sub_info):
    global_var.tof_counts = 0
    global_var.distance = sub_info
    # print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))


# =============================================================================
if __name__ == '__main__':
    # 测试基础功能
    # test_func()

    # 读取配置文件
    with open('config.yaml', 'rb') as f:
        cfg = yaml.full_load(f)
    if cfg is not None:
        config = {
            'conn_type': cfg['conn_type'],
            'green_lower': tuple(cfg['color_range']['green_lower']),
            'green_upper': tuple(cfg['color_range']['green_upper']),
            'red_lower': tuple(cfg['color_range']['red_lower']),
            'red_upper': tuple(cfg['color_range']['red_upper']),
            'blue_lower': tuple(cfg['color_range']['blue_lower']),
            'blue_upper': tuple(cfg['color_range']['blue_upper']),
            'circle_area_min': cfg['area_range']['circle_min'],
            'circle_area_max': cfg['area_range']['circle_max'],
            'rect_area_min': cfg['area_range']['rect_min'],
            'rect_area_max': cfg['area_range']['rect_max'],
            'circle_ignore_edge': cfg['size_range']['circle_ignore_edge'],
            'rect_ignore_edge': cfg['size_range']['rect_ignore_edge'],
            'ball_actual_radius': cfg['size_range']['ball_actual_radius'] / 100,
            'box_actual_width_min': cfg['size_range']['box_actual_width_min'] / 100,
            'box_actual_width_max': cfg['size_range']['box_actual_width_max'] / 100,
            'box_actual_height_min': cfg['size_range']['box_actual_height_min'] / 100,
            'box_actual_height_max': cfg['size_range']['box_actual_height_max'] / 100,
            'tof_distance_min': cfg['ctrl_param']['tof_distance_min'] * 10,
            'wheel_speed': cfg['ctrl_param']['wheel_speed'],
            'ball_offset': cfg['ctrl_param']['ball_offset'],
            'box_offset': cfg['ctrl_param']['box_offset'],
            'forward_rate': cfg['ctrl_param']['forward_rate']
        }
    else:
        config = {
            'conn_type': 'ap',
            'green_lower': global_var.GREEN_LOWER,
            'green_upper': global_var.GREEN_UPPER,
            'red_lower': global_var.RED_LOWER,
            'red_upper': global_var.RED_UPPER,
            'blue_lower': global_var.BLUE_LOWER,
            'blue_upper': global_var.BLUE_UPPER,
            'circle_area_min': global_var.CIRCLE_AREA_MIN,
            'circle_area_max': global_var.CIRCLE_AREA_MAX,
            'rect_area_min': global_var.RECT_AREA_MIN,
            'rect_area_max': global_var.RECT_AREA_MAX,
            'circle_ignore_edge': global_var.CIRCLE_IGNORE_EDGE,
            'rect_ignore_edge': global_var.RECT_IGNORE_EDGE,
            'ball_actual_radius': global_var.BALL_ACTUAL_RADIUS,
            'box_actual_width_min': global_var.BOX_ACTUAL_WIDTH_MIN,
            'box_actual_width_max': global_var.BOX_ACTUAL_WIDTH_MAX,
            'box_actual_height_min': global_var.BOX_ACTUAL_HEIGHT_MIN,
            'box_actual_height_max': global_var.BOX_ACTUAL_HEIGHT_MAX,
            'tof_distance_min': global_var.TOF_DISTANCE_MIN,
            'wheel_speed': global_var.WHEEL_SPEED,
            'ball_offset': global_var.BALL_OFFSET,
            'box_offset': global_var.BOX_OFFSET,
            'forward_rate': global_var.FORWARD_RATE
        }

    # 初始化EP对象
    ep_robot = robot.Robot()
    if config['conn_type'] == 'ap':
        ep_robot.initialize(conn_type='ap')
    else:
        ep_robot.initialize(conn_type='sta')
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_sensor = ep_robot.sensor

    # 订阅距离传感器测量的距离信息
    ep_sensor.sub_distance(freq=20, callback=sub_data_handler)

    # 机械臂回中
    ep_arm.recenter().wait_for_completed()
    ARM_POS = POS_MID
    ep_arm.moveto(x=180, y=ARM_POS).wait_for_completed()
    ep_gripper.open(power=50)
    time.sleep(1.5)
    ep_gripper.pause()

    # 按顺时针持续旋转
    w_speed = config['wheel_speed']
    ep_chassis.drive_wheels(w1=-w_speed, w2=w_speed, w3=w_speed, w4=-w_speed)

    # 开启图像检测线程
    thread_detect = ImageDetect(ep_camera, config)
    thread_detect.start()

    # 开始网球检测及运动控制
    noFound = 0
    while global_var.run_state:
        global_var.tof_counts += 1
        if global_var.tof_counts > 50:
            # 深度传感器一段时间未上报数据, 需要重新订阅
            ep_sensor.unsub_distance()
            ep_sensor.sub_distance(freq=20, callback=sub_data_handler)

        if global_var.distance[0] < config['tof_distance_min']:
            # 前方深度传感器距离小于170mm
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            ep_chassis.move(x=-0.3, y=0.3, z=0, xy_speed=0.5).wait_for_completed()
            ep_chassis.drive_wheels(w1=-20, w2=20, w3=20, w4=-20)
            global_var.distance = [1000, 1000, 1000, 1000]

        global_var.mutex_step.acquire()
        step = global_var.step
        global_var.mutex_step.release()

        forward, lateral, horizontal_degree, offset = (0.0, 0.0, 0.0, 0.3)
        if step == 1:
            # 获取当前检测到的网球图像定位向量
            global_var.mutex_target.acquire()
            forward, lateral, horizontal_degree = global_var.target_ball
            global_var.mutex_target.release()
            offset = config['ball_offset']  # 修正偏差

        elif step == 2:
            # 获取当前检测到的盒子图像定位向量
            global_var.mutex_target.acquire()
            forward, lateral, horizontal_degree = global_var.target_box
            global_var.mutex_target.release()
            offset = config['box_offset']   # 修正偏差

        if (forward is not None) and (forward != 0.0):
            noFound = 0
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

            if abs(horizontal_degree) > 2.8:
                ep_chassis.move(x=0, y=0, z=-horizontal_degree + 3.4, z_speed=10).wait_for_completed()

            if forward > offset:
                if (step == 1 and ARM_POS == POS_LOW) or (step == 2 and ARM_POS == POS_MID):
                    ep_chassis.move(x=forward * config['forward_rate'], y=0, z=0, xy_speed=0.5).wait_for_completed()
                else:
                    ep_chassis.move(x=forward * config['forward_rate'], y=0, z=0, xy_speed=0.65).wait_for_completed()
                    if step == 1 and ARM_POS != POS_LOW:
                        ARM_POS = POS_LOW
                        ep_arm.moveto(y=ARM_POS).wait_for_completed()
                    if step == 2 and ARM_POS != POS_MID:
                        ARM_POS = POS_MID
                        ep_arm.moveto(y=ARM_POS).wait_for_completed()
            elif abs(horizontal_degree) > 5.0:
                ep_chassis.move(x=0, y=0, z=-horizontal_degree / 2.0, z_speed=10).wait_for_completed()
            else:
                # 微调角度
                # ep_chassis.move(x=0, y=0, z=-horizontal_degree, z_speed=10).wait_for_completed()

                if step == 1:   # Step1: 检测到网球并移动到位
                    # 机械臂下降到底
                    if ARM_POS != POS_LOW:
                        ARM_POS = POS_LOW
                        ep_arm.moveto(y=ARM_POS).wait_for_completed()

                    # 前进7cm
                    ep_chassis.move(x=0.07, y=0, z=0, xy_speed=0.5).wait_for_completed()

                    # 机械臂前移8cm
                    ep_arm.move(x=80).wait_for_completed()

                    # 抓取网球
                    ep_gripper.close(power=50)
                    time.sleep(1)
                    ep_gripper.pause()
                    ep_chassis.move(x=-0.1, y=0, z=0, xy_speed=0.5).wait_for_completed()

                    # 重新获取当前检测到的网球图像定位向量
                    global_var.mutex_target.acquire()
                    forward, lateral, horizontal_degree = global_var.target_ball
                    global_var.mutex_target.release()

                    # 如果因为碰撞等原因没有抓取到, 需要重新检测
                    if (forward is None) or (forward > offset-0.03 or horizontal_degree > 2.0):
                        ep_gripper.open(power=50)
                        time.sleep(1.5)
                        ep_gripper.pause()
                        continue

                    # 机械臂稍微上抬，然后后移5cm
                    ARM_POS = POS_HIGH
                    ep_arm.move(y=ARM_POS).wait_for_completed()
                    ep_arm.move(x=-50).wait_for_completed()

                    global_var.mutex_step.acquire()
                    global_var.step = 2
                    global_var.mutex_step.release()

                    # 清空当前记录的目标图像向量
                    global_var.mutex_target.acquire()
                    global_var.target_box = (None, None, None)
                    global_var.mutex_target.release()

                elif step == 2:  # Step2: 检测到盒子并移动到位
                    # 离得太近需要稍微后退, 防止机械臂碰撞
                    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                    ep_chassis.move(x=-0.1, y=0, z=0, xy_speed=0.5).wait_for_completed()

                    # 抬起前伸机械臂
                    ARM_POS = POS_TOP
                    ep_arm.moveto(x=180, y=ARM_POS).wait_for_completed()

                    # 前进21cm
                    ep_chassis.move(x=0.21, y=0, z=0, xy_speed=0.5).wait_for_completed()

                    # 放置网球
                    ep_gripper.open(power=50)
                    time.sleep(1.5)
                    ep_gripper.pause()

                    # 后退并放下机械臂
                    ep_chassis.move(x=-0.4, y=0, z=0, xy_speed=0.5).wait_for_completed()
                    ARM_POS = POS_MID
                    ep_arm.moveto(x=180, y=ARM_POS).wait_for_completed()

                    global_var.mutex_step.acquire()
                    global_var.step = 1
                    global_var.mutex_step.release()

                    # 清空当前记录的目标图像向量
                    global_var.mutex_target.acquire()
                    global_var.target_ball = (None, None, None)
                    global_var.mutex_target.release()

                ep_robot.reset()
                # 重新开始按顺时针旋转检测盒子
                ep_chassis.drive_wheels(w1=-20, w2=20, w3=20, w4=-20)

                continue
        else:
            time.sleep(0.1)
            noFound += 1
            if noFound % 30 == 0:
                # 连续30次未发现目标, 移动EP位置
                # ep_chassis.move(x=0, y=-0.3, z=0, xy_speed=0.5).wait_for_completed()
                ep_chassis.move(x=0.8, y=0, z=0, xy_speed=0.6).wait_for_completed()
                ep_chassis.drive_wheels(w1=-w_speed, w2=w_speed, w3=w_speed, w4=-w_speed)
                noFound = 0

    # 取消距离传感器的信息订阅
    ep_sensor.unsub_distance()

    # 关闭roboep连接
    ep_robot.close()
