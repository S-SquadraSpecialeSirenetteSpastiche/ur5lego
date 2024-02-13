#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

image_topic = "/image_cropped"
gazebo_topic = "/gazebo/model_states"

class ImageHandler:
    def __init__(self):
        self.received_image = False
        self.received_pose = False
        self.counter = 0
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_saver)
        self.model_states_sub = rospy.Subscriber(gazebo_topic, ModelStates, self.pose_saver)

    def image_saver(self, msg):
        if not self.received_image:
            bridge = CvBridge()

            try:
                # Convert ROS Image message to OpenCV2
                cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError:
                print('error during conversion')
            else:
                directory_path = "/home/utente/Desktop/uni/robotica/block_images"
                block_type = "X1-Y4-Z2"
                file_name = "camera_image_"
                file_path = os.path.join(directory_path, block_type, file_name) + str(self.counter) + '.jpeg'

                # Create the directory structure if it doesn't exist
                os.makedirs(os.path.join(directory_path, block_type), exist_ok=True)

                while os.path.exists(file_path):
                    self.counter += 1
                    file_path = os.path.join(directory_path, block_type, file_name) + str(self.counter) + '.jpeg'
                
                cv2.imwrite(file_path, cv2_img)
                rospy.loginfo(f"{file_path} saved")

                self.received_image = True
                self.image_sub.unregister()

    def pose_saver(self, msg):
        if not self.received_pose and self.received_image: # enter after saving the image and update the counter
            try:
                idx = msg.name.index("block1")
                position = msg.pose[idx].position
                orientation = msg.pose[idx].orientation
                
                directory_path = "/home/utente/Desktop/uni/robotica/block_position"
                block_type = "X1-Y4-Z2"
                file_name = "block_position_"
                file_path = os.path.join(directory_path, block_type, file_name) + str(self.counter) + '.txt'
                
                # Create the directory structure if it doesn't exist
                os.makedirs(os.path.join(directory_path, block_type), exist_ok=True)

                # push position and orientation in the file and save it
                with open(file_path, 'w') as file:
                    file.write("{} {} {} ".format(position.x, position.y, position.z))
                    file.write("{} {} {} {}".format(orientation.x, orientation.y, orientation.z, orientation.w))

                rospy.loginfo(f"{file_path} saved")

                self.received_pose = True
                self.model_states_sub.unregister()
                rospy.signal_shutdown('image received')
            except ValueError:
                rospy.logwarn("Model 'block1' not found in model states")

if __name__ == '__main__':
    rospy.init_node('image_listener', disable_signals=True)

    reader = ImageHandler()
    # Spin until ctrl + c
    rospy.spin()