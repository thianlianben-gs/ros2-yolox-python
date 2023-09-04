#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) Megvii, Inc. and its affiliates.

# ROS2 rclpy -- Ar-Ray-code 2022
import argparse
import os

import cv2
import numpy as np

import onnxruntime

from yolox.data.data_augment import preproc as preprocess
from yolox.data.datasets import COCO_CLASSES
from yolox.utils import mkdir, demo_postprocess, vis

from .yolox_ros_py_utils.utils import yolox_py

# ROS2 =====================================
import rclpy
from rclpy.node import Node

import base64
import sys

from std_msgs.msg import Header, Byte, String, UInt8
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from bboxes_ex_msgs.msg import BoundingBoxes
from bboxes_ex_msgs.msg import BoundingBox

from rclpy.qos import qos_profile_sensor_data

# from darkself.net_ros_msgs.msg import BoundingBoxes
# from darkself.net_ros_msgs.msg import BoundingBox



def nms(boxes, scores, nms_thr):
    """Single class NMS implemented in Numpy."""
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(ovr <= nms_thr)[0]
        order = order[inds + 1]

    return keep

def multiclass_nms(boxes, scores, nms_thr, score_thr):
        """Multiclass NMS implemented in Numpy"""
        final_dets = []
        num_classes = scores.shape[1]
        for cls_ind in range(num_classes):
            cls_scores = scores[:, cls_ind]
            valid_score_mask = cls_scores > score_thr

            if valid_score_mask.sum() == 0:
                continue
            else:
                valid_scores = cls_scores[valid_score_mask]
                valid_boxes = boxes[valid_score_mask]
                keep = nms(valid_boxes, valid_scores, nms_thr)
                if len(keep) > 0:
                    cls_inds = np.ones((len(keep), 1)) * cls_ind
                    dets = np.concatenate([valid_boxes[keep], valid_scores[keep, None], cls_inds], 1)
                    final_dets.append(dets)
        if len(final_dets) == 0:
            return None
        return np.concatenate(final_dets, 0)

class yolox_ros(yolox_py):


    def __init__(self) -> None:

        # ROS2 init
        super().__init__('yolox_ros', load_params=True)

        if (self.imshow_isshow):
            cv2.namedWindow("YOLOX")
        
        self.bridge = CvBridge()

        self.rgb_means = (0.485, 0.456, 0.406)
        self.std = (0.229, 0.224, 0.225)
        
        self.pub = self.create_publisher(BoundingBoxes,"bounding_boxes", 10)
        self.sub = self.create_subscription(Image,"image_raw",self.imageflow_callback, self.qos_image_sub)
        self.end_sub = self.create_subscription(String, "end_system",self.end_system_callback, self.qos_image_sub)

        self.pub_image = self.create_publisher(CompressedImage, "image_publisher", 10)

    def end_system_callback(self, msg:String) -> None:
        try:
            self.get_logger().info("Shutting down ROS node due to a condition.")
            self.destroy_node()  # Trigger node shutdown
            rclpy.shutdown() 
            sys.exit(0)
        except Exception as e:
            self.get_logger().info(f'Error: {e}')
            pass

    def imageflow_callback(self,msg:Image) -> None:
        try:
            # fps start
            start_time = cv2.getTickCount()
            bboxes = BoundingBoxes()
            origin_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")

            # preprocess
            img, self.ratio = preprocess(origin_img, self.input_shape)

            # session = onnxruntime.InferenceSession(self.model_path)
            session = onnxruntime.InferenceSession("/home/ben/ros2_ws/src/yolox_ros/weights/yolox_nano.onnx")

            ort_inputs = {session.get_inputs()[0].name: img[None, :, :, :]}
            output = session.run(None, ort_inputs)
            
            predictions = demo_postprocess(output[0], self.input_shape, p6=self.with_p6)[0]

            boxes = predictions[:, :4]

            scores = predictions[:, 4:5] * predictions[:, 5:]

            boxes_xyxy = np.ones_like(boxes)
            boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2]/2.
            boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3]/2.
            boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2]/2.
            boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3]/2.
            boxes_xyxy /= self.ratio
            dets = multiclass_nms(boxes_xyxy, scores, nms_thr=self.nms_th, score_thr=self.conf)

            if dets is not None:
                self.final_boxes, self.final_scores, self.final_cls_inds = dets[:, :4], dets[:, 4], dets[:, 5]
                origin_img = vis(origin_img, self.final_boxes, self.final_scores, self.final_cls_inds,
                         conf=self.conf, class_names=COCO_CLASSES)
                         
                msg = CompressedImage()
                msg.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg', origin_img)[1]).tostring()
                self.pub_image.publish(msg)

            end_time = cv2.getTickCount()
            time_took = (end_time - start_time) / cv2.getTickFrequency()

            # rclpy log FPS
            self.get_logger().info(f'FPS: {1 / time_took}')
            
            try:
                bboxes = self.yolox2bboxes_msgs(dets[:, :4], self.final_scores, self.final_cls_inds, COCO_CLASSES, msg.header, origin_img)
                if (self.imshow_isshow):
                    cv2.imshow("YOLOX",origin_img)
                    cv2.waitKey(1)

            except:
                if (self.imshow_isshow):
                    cv2.imshow("YOLOX",origin_img)
                    cv2.waitKey(1)

            self.pub.publish(bboxes)
            # img_data = self.bridge.cv2_to_imgmsg(origin_img, encoding="bgr8")
            # self.pub_image.publish(img_data)
            

        except Exception as e:
            self.get_logger().info(f'Error: {e}')
            pass

def ros_main(args = None):
    rclpy.init(args=args)
    ros_class = yolox_ros()

    try:
        rclpy.spin(ros_class)
    except KeyboardInterrupt:
        pass
    finally:
        ros_class.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    ros_main()