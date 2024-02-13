#!/usr/bin/env python

import argparse
import csv
import math
import os
import platform
import sys
from pathlib import Path
import numpy as np

import torch
from ultralytics.utils.plotting import Annotator, colors
from models.common import DetectMultiBackend

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ur5lego.msg import Pose

# from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (
    Profile,
    check_img_size,
    non_max_suppression,
    scale_boxes
)
from utils.augmentations import letterbox
from utils.torch_utils import select_device, smart_inference_mode
from script.pointcloud import PointCloudHandler
import rospkg

bridge = CvBridge()

rospack = rospkg.RosPack()
# Check if a specific package exists
package_name = "ur5lego"
package_path = rospack.get_path(package_name)

weights_path = os.path.join(package_path, 'data/best.pt')

@smart_inference_mode()
def run(msg):
    imgsz = (416, 416)
    
    # Dataloader
    try:
        im0s = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Error processing the image: %s", str(e))
    else:
        # Load model
        device = select_device("")
        model = DetectMultiBackend(weights_path, device=device)
        stride, names = model.stride, model.names
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Run inference
        model.warmup(imgsz=(1, 3, *imgsz))  # warmup
        seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))
        
        with dt[0]:
            im = letterbox(im0s, imgsz, stride=stride)[0]  # padded resize
            im = im.transpose((2, 0, 1))[::-1] # HWC to CHW, BGR to RGB
            im = np.ascontiguousarray(im)
            im = torch.from_numpy(im).to(model.device)
            im = im.float()
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
    
        # Inference
        with dt[1]:
            pred = model(im, augment=False, visualize=False)

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres = 0.40, iou_thres = 0.25, max_det=1000)

        boxes = []
        labels = []

        # Process predictions
        for i, det in enumerate(pred):
            seen += 1

            im0 = im0s.copy()

            annotator = Annotator(im0, line_width=3, example=str(names))

            centers = []

            if len(det):
                # det: x_topleft, y_topleft, x_bottomright, y_bottomright, confidence, class 
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    box = torch.tensor(xyxy).view(4).view(-1).tolist()
                    
                    centers.append(((box[2] + box[0]) / 2.0, (box[3] + box[1]) / 2))
                    c = int(cls)  # integer class
                    label = f"{names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))

                    boxes.append([int(i) for i in box])
                    labels.append(c) # names[c] if the name is required
    
            # Stream results
            # im0 = annotator.result()
            # p = "detection"
            # if platform.system() == "Linux" and p not in windows:
            #     windows.append(p)
            #     cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
            #     cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
            # cv2.imshow(str(p), im0)
            # cv2.waitKey(0)

    if len(boxes) > 0:
        print("entrato")
        ph = PointCloudHandler(names)
        ph.detect_block(boxes, labels)

    rospy.signal_shutdown('detection finished')

if __name__ == "__main__":
    try:
        rospy.init_node('lego_detector')

        # Subscribe to the image topic
        image_sub = rospy.Subscriber('/image_cropped', Image, run)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
