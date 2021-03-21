# -*-coding:utf-8-*-

import cv2
import measure
import threading
import global_var
from robomaster import camera
from typing import Tuple, List, Optional


class ImageDetect(threading.Thread):
    def __init__(self, ep_camera, cfg):
        super().__init__()
        self._camera = ep_camera
        self._cfg = cfg

    # =============================================================================
    # 多边形拟合封闭曲线, 返回近似曲线点集和图形面积
    # -----------------------------------------------------------------------------
    @staticmethod
    def contour_analysis(cnt) -> Tuple[int, int]:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(cnt)
        return len(approx), area

    # =============================================================================
    # 寻找面积最大点集圆, 拟合多边形顶点数至少>8
    # -----------------------------------------------------------------------------
    def biggest_circle_cnt(self, cnts: List) -> Tuple[List, int]:
        found_cnt = None
        found_edges = 0
        found_area = 0

        for cnt in cnts:
            edges, area = self.contour_analysis(cnt)
            if edges > 8 \
                    and self._cfg['circle_area_min'] < area < self._cfg['circle_area_max'] \
                    and edges > found_edges \
                    and area > found_area:
                (x, y), pixel_radius = cv2.minEnclosingCircle(cnt)
                l, t, w, h = cv2.boundingRect(cnt)
                # 屏幕左右边缘的点集忽略
                if (self._cfg['circle_ignore_edge'] < x < measure.HORIZONTAL_PIXELS - self._cfg['circle_ignore_edge']) \
                        and (0.8 < w / h < 1.4):
                    found_edges = edges
                    found_area = area
                    found_cnt = cnt

        return found_cnt, found_area

    # =============================================================================
    # 寻找面积最大点集矩形, 拟合多边形顶点数>4
    # -----------------------------------------------------------------------------
    def biggest_rect_cnt(self, cnts: List) -> Tuple[List, int]:
        found_cnt = None
        found_edges = 0
        found_area = 0

        for cnt in cnts:
            edges, area = self.contour_analysis(cnt)
            if edges >= 4 \
                    and self._cfg['rect_area_min'] < area < self._cfg['rect_area_max'] \
                    and edges > found_edges \
                    and area > found_area:
                x, y, w, h = cv2.boundingRect(cnt)
                # 屏幕左右边缘的点集忽略
                if (self._cfg['rect_ignore_edge'] < x < measure.HORIZONTAL_PIXELS - self._cfg['rect_ignore_edge'] - w) \
                        and (w / h > self._cfg['box_actual_width_min'] / self._cfg['box_actual_height_max'] - 0.4) \
                        and (w / h < self._cfg['box_actual_width_max'] / self._cfg['box_actual_height_min'] + 0.4):
                    found_edges = edges
                    found_area = area
                    found_cnt = cnt

        return found_cnt, found_area

    # =============================================================================
    # 检测网球并标记
    # -----------------------------------------------------------------------------
    def detect_tennis(self, frame, color_lower, color_upper) -> Optional[Tuple[float, float, float]]:
        processed = cv2.GaussianBlur(frame, (11, 11), 0)
        processed = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(processed, color_lower, color_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 获取最大面积圆形点集
        ball_cnt, ball_area = self.biggest_circle_cnt(cnts)
        if ball_cnt is None:
            cv2.putText(frame, 'tof: %.1f cm' % (global_var.distance[0] / 10), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 0, 255), 2)
            cv2.putText(frame, 'no tennis detected', (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow('EV-RoboTN', frame)
            cv2.waitKey(1)
            return None, None, None

        (x, y), pixel_radius = cv2.minEnclosingCircle(ball_cnt)
        distance = measure.pinhole_distance(self._cfg['ball_actual_radius'], pixel_radius)
        forward, lateral, horizontal_degree = measure.distance_decomposition(x, distance)
        cv2.circle(frame, (int(x), int(y)), int(pixel_radius), (0, 255, 0), 2)
        cv2.circle(frame, (int(x), int(y)), 1, (0, 0, 255), 2)
        cv2.putText(frame, 'tof: %.1f cm' % (global_var.distance[0] / 10), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 0, 255), 2)
        cv2.putText(frame, 'forward: %.1f cm' % (forward * 100), (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, 'lateral: %.1f cm' % (lateral * 100), (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, 'degree: %.1f' % horizontal_degree, (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, 'area: %d pixels' % ball_area, (50, 230), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('EV-RoboTN', frame)
        key = cv2.waitKey(1)
        if key & 0xFF == 27:    # ESC
            global_var.run_state = False

        return forward, lateral, horizontal_degree

    # =============================================================================
    # 检测盒子并标记
    # -----------------------------------------------------------------------------
    def detect_box(self, frame, color_lower, color_upper) -> Optional[Tuple[float, float, float]]:
        processed = cv2.GaussianBlur(frame, (11, 11), 0)
        processed = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(processed, color_lower, color_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 获取最大面积矩形点集
        rect_cnt, rect_area = self.biggest_rect_cnt(cnts)
        if rect_cnt is None:
            cv2.putText(frame, 'tof: %.1f cm' % (global_var.distance[0] / 10), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 0, 255), 2)
            cv2.putText(frame, 'no box detected', (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow('EV-RoboTN', frame)
            cv2.waitKey(1)
            return None, None, None

        x, y, w, h = cv2.boundingRect(rect_cnt)
        if w < h:
            # 高度比较大时, 以横向宽度作为计算依据
            distance = measure.pinhole_distance((self._cfg['box_actual_width_max'] +
                                                 self._cfg['box_actual_width_min']) / 2.2, w)
        else:
            # 宽度比较大时, 以纵向高度作为计算依据
            distance = measure.pinhole_distance((self._cfg['box_actual_height_max'] +
                                                 self._cfg['box_actual_height_min']) / 2.2, h)

        forward, lateral, horizontal_degree = measure.distance_decomposition(x + w / 2, distance)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(frame, 'tof: %.1f cm' % (global_var.distance[0] / 10), (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 0, 255), 2)
        cv2.putText(frame, 'forward: %.1f cm' % (forward * 100), (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, 'lateral: %.1f cm' % (lateral * 100), (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, 'degree: %.1f' % horizontal_degree, (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, 'area: %d pixels' % rect_area, (50, 230), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('EV-RoboTN', frame)
        key = cv2.waitKey(1)
        if key & 0xFF == 27:    # ESC
            global_var.run_state = False

        return forward, lateral, horizontal_degree

    # =============================================================================
    # 启动EP图像回传并检测目标
    # -----------------------------------------------------------------------------
    def run(self):
        # 图像回传显示
        self._camera.start_video_stream(display=False, resolution=camera.STREAM_720P)

        while global_var.run_state:
            try:
                # 捕获一帧图像
                img = self._camera.read_cv2_image(strategy="newest", timeout=3)

                global_var.mutex_step.acquire()
                step = global_var.step
                global_var.mutex_step.release()

                if step == 1:
                    # 检测并标记网球
                    forward, lateral, horizontal_degree = \
                        self.detect_tennis(img, self._cfg['green_lower'], self._cfg['green_upper'])
                    # 将图像定位向量输出
                    global_var.mutex_target.acquire()
                    global_var.target_ball = (forward, lateral, horizontal_degree)
                    global_var.target_box = (None, None, None)
                    global_var.mutex_target.release()

                elif step == 2:
                    # 检测并标记盒子
                    forward, lateral, horizontal_degree = \
                        self.detect_box(img, self._cfg['blue_lower'], self._cfg['blue_upper'])
                    # 将图像定位向量输出
                    global_var.mutex_target.acquire()
                    global_var.target_ball = (None, None, None)
                    global_var.target_box = (forward, lateral, horizontal_degree)
                    global_var.mutex_target.release()

                else:
                    global_var.mutex_target.acquire()
                    global_var.target_ball = (None, None, None)
                    global_var.target_box = (None, None, None)
                    global_var.mutex_target.release()

            except Exception as e:
                print(f"msg get exception = {e}")
                # 关闭图像窗口
                # cv2.destroyAllWindows()

                # 重启图像回传
                self._camera.stop_video_stream()
                self._camera.start_video_stream(display=False, resolution=camera.STREAM_720P)

        # 停止图像回传
        self._camera.stop_video_stream()
        cv2.destroyAllWindows()
