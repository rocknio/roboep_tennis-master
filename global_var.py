# -*-coding:utf-8-*-

import threading

# GREEN_LOWER = (29, 90, 90)
# GREEN_UPPER = (64, 255, 255)
GREEN_LOWER = (32, 90, 90)
GREEN_UPPER = (64, 255, 220)
RED_LOWER = (0, 90, 40)
RED_UPPER = (16, 255, 220)
BLUE_LOWER = (90, 90, 60)
BLUE_UPPER = (140, 255, 220)

CIRCLE_AREA_MIN = 1000
CIRCLE_AREA_MAX = 32000
RECT_AREA_MIN = 5000
RECT_AREA_MAX = 250000
CIRCLE_IGNORE_EDGE = 200
RECT_IGNORE_EDGE = 50
BALL_ACTUAL_RADIUS = 0.0325     # 网球直径6.5cm
BOX_ACTUAL_WIDTH_MIN = 0.19     # 最小宽度19cm
BOX_ACTUAL_WIDTH_MAX = 0.28     # 最大宽度28cm
BOX_ACTUAL_HEIGHT_MIN = 0.16    # 最小高度16cm
BOX_ACTUAL_HEIGHT_MAX = 0.19    # 最大高度19cm

TOF_DISTANCE_MIN = 170  # 深度探测最小限定距离170mm
WHEEL_SPEED = 30        # EP底盘默认旋转速度30度/秒
BALL_OFFSET = 0.29      # 网球前向距离修正29cm
BOX_OFFSET = 0.40       # 盒子前向距离修正40cm
FORWARD_RATE = 0.83     # 前向运动趋近比率

run_state = True        # 持续运行状态, 按ESC后变为False
step = 1                # 运行步骤: 1-抓球, 2-投球
tof_counts = 0
distance = [1000, 1000, 1000, 1000]
target_ball = (None, None, None)
target_box = (None, None, None)

mutex_step = threading.Lock()
mutex_target = threading.Lock()
