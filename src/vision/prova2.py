#!/usr/bin/env python

import argparse
import csv
import os
import platform
import sys
from pathlib import Path
import numpy as np

import torch
from ultralytics.utils.plotting import Annotator, colors, save_one_box
from models.common import DetectMultiBackend

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from ur5lego.msg import Pose
import sensor_msgs.point_cloud2 as pc2

from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (
    LOGGER,
    Profile,
    check_file,
    check_img_size,
    check_imshow,
    check_requirements,
    colorstr,
    cv2,
    increment_path,
    non_max_suppression,
    print_args,
    scale_boxes,
    strip_optimizer,
    xyxy2xywh,
)
from utils.augmentations import letterbox
from utils.torch_utils import select_device, smart_inference_mode

FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]  # ur5lego root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

bridge = CvBridge()
boxes = []
labels = []
pointcloud_sent = False

@smart_inference_mode()
def run(msg):
    
    weights=ROOT / "data" / "better.pt"
    data=ROOT / "data" / "data.yaml"
    imgsz = (640, 640)
    
    # Dataloader
    try:
        im0s = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Error processing the image: %s", str(e))
    else:
        # Load model
        device = select_device("")
        model = DetectMultiBackend(weights, device=device, data=data)
        stride, names, pt = model.stride, model.names, model.pt
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
            pred = non_max_suppression(pred, conf_thres = 0.60, iou_thres = 0.75, max_det=1000)

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):
            seen += 1
            
            im0 = im0s.copy()

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            
            annotator = Annotator(im0, line_width=3, example=str(names))

            centers = []

            if len(det):
                # det: x_topleft, y_topleft, x_bottomright, y_bottomright, confidence, class 
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class

                global boxes, labels

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    box = torch.tensor(xyxy).view(4).view(-1).tolist()
                    
                    centers.append(((box[2] + box[0]) / 2.0, (box[3] + box[1]) / 2))
                    c = int(cls)  # integer class
                    label = f"{names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))

                    boxes.append(box)
                    labels.append(c)
    
            # Stream results
            im0 = annotator.result()
            p = "detection"
            if platform.system() == "Linux" and p not in windows:
                windows.append(p)
                cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
            cv2.imshow(str(p), im0)
            cv2.waitKey(0)  # 1 millisecond

    handle_center()

    rospy.signal_shutdown('detection finished')

def handle_center():
    global pointcloud_sent

    pointcloud_sub = rospy.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2, send_position)

    while not pointcloud_sent:
        rospy.sleep(0.1)

    pointcloud_sub.unregister()

def send_position(msg):
    global pointcloud_sent

    center_pub = rospy.Publisher('/ciao_angela', Pose, queue_size=10)
    center_pub.publish(Pose())

    points = pc2.read_points(msg)

    # print("size ({}, {})".format(msg.width, msg.height))
    # print(boxes[0])

    width = msg.width
    height = msg.height

    # make this procedure work for all the detected boxes

    center = (int((boxes[0][0] + boxes[0][2]) / 2), int((boxes[0][1] + boxes[0][3]) / 2))

    # Iterate over points similar to image pixels
    for u in range(height):
        for v in range(width):
            point = next(points)
            # Access point data fields (e.g., x, y, z, intensity)
            x, y, z, intensity = point
            
            #maybe change the detection part in an external function and call it from image_cropper
            x_offset = 672
            y_offset = 399

            # if v > boxes[0][0] and v < boxes[0][2] and u > boxes[0][1] and u < boxes[0][3]:
            if v == center[0] + x_offset and u == center[1] + y_offset:
                pose_msg = Pose()

                pose_msg.position.x = x
                pose_msg.position.y = y
                pose_msg.position.z = z
                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                pose_mgs.legoType = labels[0]

                center_pub.publish(pose_msg)
                print("Point ({}, {}): ({}, {}, {}, {})".format(u, v, x, y, z, intensity))
    
    pointcloud_sent = True

if __name__ == "__main__":
    try:
        rospy.init_node('lego_detector')

        # Subscribe to the image topic
        # image_sub = rospy.Subscriber('/ur5/zed_node/left_raw/image_raw_color', Image, run)
        image_sub = rospy.Subscriber('/image_cropped', Image, run)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
