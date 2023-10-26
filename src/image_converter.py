#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from PIL import Image as im
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

bridge = CvBridge()

img = np.zeros((480, 640, 3))
fig, ax = plt.subplots()

def image_converter(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", x)
    # rospy.loginfo(msg.data)

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print('error during conversion')
    else:
        # TODO: gestione stampa/salvataggio immagine
        cv2.imwrite('camera_image.jpeg', cv2_img)
        cv2.imshow("Image window", cv2_img)
        cv2.waitKey(1000)
    # rospy.signal_shutdown('image received')

def main():
    # global img
    # img = ax.imshow(img, cmap='viridis')
    rospy.init_node('image_listener', disable_signals=True)
    # Define your image topic
    image_topic = "/z_base_camera/camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_converter)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()