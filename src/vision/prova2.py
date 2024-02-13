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
from script.pointcloud import PointCloudHandler

FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]  # ur5lego root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

bridge = CvBridge()
boxes = []
labels = []
pointcloud_sent = False
im0_clone = None

@smart_inference_mode()
def run(msg):
    
    # weights=ROOT / "data" / "better.pt"
    weights="/home/utente/Desktop/uni/robotica/ur5lego/datasetGigante/60_epoche/best.pt"
    data=ROOT / "data" / "data.yaml"
    imgsz = (416, 416)
    
    # Dataloader
    try:
        im0s = bridge.imgmsg_to_cv2(msg, "bgr8")
        global im0_clone
        im0_clone = im0s.copy()
    except Exception as e:
        rospy.logerr("Error processing the image: %s", str(e))
    else:
        # Load model
        device = select_device("")
        model = DetectMultiBackend(weights, device=device) #, data=data)
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
            pred = non_max_suppression(pred, conf_thres = 0.40, iou_thres = 0.25, max_det=1000)

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        global boxes, labels

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
            im0 = annotator.result()
            p = "detection"
            if platform.system() == "Linux" and p not in windows:
                windows.append(p)
                cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
            cv2.imshow(str(p), im0)
            cv2.waitKey(0)  # 1 millisecond

    if len(boxes) > 0 and not pointcloud_sent:
        print("entrato")
        ph = PointCloudHandler(names)
        ph.detect_block(boxes, labels)
        #handle_center()

    rospy.signal_shutdown('detection finished')

def handle_center():
    global pointcloud_sent

    pointcloud_sub = rospy.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2, send_position)

    while not pointcloud_sent:
        rospy.sleep(1000.0)

    pointcloud_sub.unregister()

def send_position(msg):
    global pointcloud_sent

    center_pub = rospy.Publisher('/ciao_angela', Pose, queue_size=10)
    center_pub.publish(Pose())

    # make this procedure work for all the detected boxes

    center = (int((boxes[0][0] + boxes[0][2]) / 2), int((boxes[0][1] + boxes[0][3]) / 2))

    #center = (520, 940)

    center_pointcloud = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[center])

    pose_msg = Pose()

    for pixel in center_pointcloud:
        # print(pixel)
        pose_msg.position.x = pixel[0]
        pose_msg.position.y = pixel[1]
        pose_msg.position.z = pixel[2]
    
    depth_image = []

    for x in range(int(boxes[0][0]), int(boxes[0][2]) + 1):
        for y in range(int(boxes[0][1]), int(boxes[0][3]) + 1):
    # for x in range(510, 530 + 1):
    #     for y in range(930, 950 + 1):
            pixels = pc2.read_points(msg, field_names=['x', 'y', 'z'], uvs=[(x, y)])
            
            for pixel in pixels:
                depth_image.append((pixel[0], pixel[1], pixel[2]))

    depth_image = [ np.sqrt(x**2 + y**2 + z**2) for x, y, z in depth_image ]

    depth_image = np.array(depth_image).reshape((int(boxes[0][2] - boxes[0][0]) + 1, int(boxes[0][3] - boxes[0][1]) + 1))

    depth_image = np.where(depth_image < depth_image.max() - 0.005, 255, 0).astype(np.uint8)

    for row in depth_image:
        print("[")
        for pixel_depth in row:
            print(pixel_depth, end=' ')
        print("]")
    # print(depth_image)

    # very bad with this method for finding the threshold
    # try some sort of pca for understand the behaviour of the point
    # with lego rotated only on z axis should work quite well
    depth_image, _ = cv2.findContours(depth_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    try:                                                                                            
        depth_image = depth_image[np.argmax([len(x) for x in depth_image], axis=0)]              
    except ValueError:
        return
    
    lego_siluette = cv2.minAreaRect(depth_image)

    box = cv2.boxPoints(lego_siluette) + (boxes[0][0], boxes[0][1]) # + top left corner
    box = np.intp(box)

    global im0_clone

    cv2.drawContours(im0_clone, [box], 0, (0, 0, 255), 2)

    angle = lego_siluette[2] + 90 if lego_siluette[1][0] < lego_siluette[1][1] else lego_siluette[2]
    angle %= 180
    angle = 180 - angle
    yaw = angle/180 * math.pi
    
    print(yaw)

    pose_msg.orientation.x = 0.0
    pose_msg.orientation.y = 0.0
    pose_msg.orientation.z = yaw
    pose_msg.legoType = labels[0]

    pointcloud_sent = True
    
    # cv2.imshow("pointcloud", im0_clone)
    # cv2.waitKey(0)

    # print("size ({}, {})".format(msg.width, msg.height))
    # print(boxes[0])

    width = msg.width
    height = msg.height

    # Iterate over points similar to image pixels
    # for u in range(height):
    #     for v in range(width):
    #         point = next(points)
    #         # Access point data fields (e.g., x, y, z, intensity)
    #         x, y, z, intensity = point
            
    #         #maybe change the detection part in an external function and call it from image_cropper
    #         x_offset = 672
    #         y_offset = 399

    #         # if v > boxes[0][0] and v < boxes[0][2] and u > boxes[0][1] and u < boxes[0][3]:
    #         if v == center[0] + x_offset and u == center[1] + y_offset:
    #             pose_msg = Pose()

    #             pose_msg.position.x = x
    #             pose_msg.position.y = y
    #             pose_msg.position.z = z
    #             pose_msg.orientation.x = 0.0
    #             pose_msg.orientation.y = 0.0
    #             pose_msg.orientation.z = 0.0
    #             pose_msg.legoType = labels[0]

    #             center_pub.publish(pose_msg)
    #             print("Point ({}, {}): ({}, {}, {}, {})".format(u, v, x, y, z, intensity))

if __name__ == "__main__":
    try:
        rospy.init_node('lego_detector')

        # Subscribe to the image topic
        #image_sub = rospy.Subscriber('/ur5/zed_node/left_raw/image_raw_color', Image, run)
        image_sub = rospy.Subscriber('/image_cropped', Image, run)

        # -0.87736952, 0.20241742, -0.45750925

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
