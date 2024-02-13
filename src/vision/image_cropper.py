#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageCropper:
    def __init__(self):
        rospy.init_node('image_cropper_node')

        self.bridge = CvBridge()

        # Subscribe to the image topic you want to crop
        self.image_sub = rospy.Subscriber('/ur5/zed_node/left_raw/image_raw_color', Image, self.image_callback)

        # Publisher for the cropped image
        self.cropped_image_pub = rospy.Publisher('/image_cropped', Image, queue_size=10)
        self.cropped_image_pub.publish(Image())

        # Define the region to crop (x, y, width, height)
        self.crop_x = 672
        self.crop_y = 399
        self.crop_width = 884
        self.crop_height = 584
        self.image_size = 640

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Crop the image
            cropped_image = cv_image[self.crop_y:self.crop_y + self.crop_height,
                                     self.crop_x:self.crop_x + self.crop_width]
            
            # cropped_image = cv2.resize(cropped_image, (self.image_size, self.image_size))

            # Convert cropped image to ROS format and publish
            cropped_image_msg = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
            self.cropped_image_pub.publish(cropped_image_msg)

        except Exception as e:
            rospy.logerr("Error processing the image: %s", str(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        cropper = ImageCropper()
        cropper.run()
    except rospy.ROSInterruptException:
        pass