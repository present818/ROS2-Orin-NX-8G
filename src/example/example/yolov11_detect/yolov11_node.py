#!/usr/bin/env python3
# encoding: utf-8
# @data:2025/05/26
# @author:aiden
# yolo目标检测(yolo target detection)
import os
import cv2
import time
import queue
import rclpy
import signal
import logging
import threading
import numpy as np
import sdk.fps as fps
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from interfaces.msg import ObjectInfo, ObjectsInfo
from example.yolov8_detect.yolov8_trt import plot_one_box, colors

logging.getLogger('ultralytics').setLevel(logging.WARNING)
MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class YoloNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.start = True
        self.running = True

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)

        self.fps = fps.FPS()  # fps计算器(FPS calculator)
        engine = self.get_parameter('engine').value
        self.conf_thresh = self.get_parameter('conf').value
        self.classes = self.get_parameter('classes').value
        task = self.get_parameter('task').value
        self.display = self.get_parameter('disaplay').value

        self.yolo = YOLO(os.path.join(MODE_PATH, engine), task=task)
        self.create_service(Trigger, '~/start', self.start_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)  # 退出玩法(exit the game)

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)

        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start yolo detect")

        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop yolo detect")

        self.start = False
        response.success = True
        response.message = "start"
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)
   
    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def image_proc(self):

        while self.running:
            try:
                image_from_queue = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            
            image = image_from_queue.copy()

            try:
                if self.start:
                    objects_info = []
                    h, w = image.shape[:2]
                    results = self.yolo(cv2.resize(image, (640, 480)), conf=self.conf_thresh, imgsz=[480, 640]) #使用的垃圾分类模型是640X480,所以要对图像进行缩放

                    for result in results:
                        obb = result.obb
                        if obb is not None and len(obb.cls) > 0:

                            xywhr_boxes = obb.xywhr.cpu().numpy().astype(int)
                            xywhr_boxes[0][1] = xywhr_boxes[0][1] - 40 # 使用的垃圾分类模型是640X480，而相机分辨率为640X400，获取的中心坐标需要往上移40个像素点                            
                            confs = np.atleast_1d(obb.conf.cpu().numpy())
                            cls_ids = np.atleast_1d(obb.cls.cpu().numpy().astype(int))

                            for i in range(len(xywhr_boxes)):
                                single_conf = confs[i]
                                
                                current_cls_item = np.atleast_1d(cls_ids[i])
                                single_cls_id = current_cls_item[0]

                                class_name = self.yolo.names[single_cls_id]
                                color = colors(single_cls_id, bgr=True)
                                plot_one_box(
                                        xywhr_boxes[i], 
                                        image,
                                        label=f"{class_name}:{single_conf:.2f}",
                                        color=color,
                                        line_thickness=3, 
                                        rotated=True)
                                
                                object_info = ObjectInfo()
                                object_info.class_name = class_name
                                object_info.box = xywhr_boxes.reshape(-1).tolist()
                                object_info.score = float(single_conf)
                                object_info.width = w
                                object_info.height = h
                                objects_info.append(object_info)

                    object_msg = ObjectsInfo()
                    object_msg.objects = objects_info
                    self.object_pub.publish(object_msg)
                else:
                    time.sleep(0.01)
            except BaseException as e:
                print('error', e)

            self.fps.update()
            result_image = self.fps.show_fps(image)
            if self.display:
                cv2.imshow('yolo', result_image)
                cv2.waitKey(1)
            ros_image = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "yolo"
            self.result_image_pub.publish(ros_image)
        else:
            time.sleep(0.01)
        rclpy.shutdown()

def main():
    node = YoloNode('yolo')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
