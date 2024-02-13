#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
import math
import argparse

# Create a quaternion from Euler angles (roll, pitch, yaw)
def euler_to_quaternion(roll, pitch, yaw):
    quaternion = Quaternion()

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    quaternion.w = cr * cp * cy + sr * sp * sy

    return quaternion

def get_model_state(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')  # Wait for the service to become available
    try:
        # Create a proxy to the service
        get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Call the service to get the model state
        response = get_model_state_proxy(model_name, '')

        # Process the response
        if response.success:
            return response
        else:
            rospy.logwarn(f"Failed to get state for '{model_name}'")
            rospy.signal_shutdown('Error')
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def call_set_model_state_service(model_name, position=None, orientation=None):
    rospy.wait_for_service('/gazebo/set_model_state')  # Wait for the service to become available
    try:
        while True:
            response = get_model_state(model_name)
            
            # Create a proxy to the service
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            
            # Create a request message for setting the model state
            state_msg = ModelState()
            state_msg.model_name = model_name
            
            # Set the position, if not provided use the old one
            if position:
                if len(position) == 3 or len(position) == 2:
                    state_msg.pose.position.x = position[0]
                    state_msg.pose.position.y = position[1]
                    state_msg.pose.position.z = 0.88 # position[2] is optional
                else:
                    rospy.loginfo("Invalid position format. Should be a 2D or 3D vector.")
                    rospy.signal_shutdown('Error')
            else:
                state_msg.pose.position = response.pose.position

            # Set the orientation, if not provided use the old one
            if orientation:
                if len(orientation) == 3:
                    state_msg.pose.orientation = euler_to_quaternion(orientation[0], orientation[1], orientation[2])
                elif len(orientation) == 4:
                    state_msg.pose.orientation.x = orientation[0]
                    state_msg.pose.orientation.y = orientation[1]
                    state_msg.pose.orientation.z = orientation[2]
                    state_msg.pose.orientation.w = orientation[3]
                else:
                    rospy.loginfo("Invalid orientation format. Should be a 3D or 4D vector.")
                    rospy.signal_shutdown('Error')
            else:
                state_msg.pose.orientation = response.pose.orientation

            # Initialize the remaining field with the old state
            state_msg.twist = response.twist

            # Call the service
            response = set_model_state(state_msg)
            
            # Check the response and perform actions accordingly
            if response.success:
                rospy.loginfo("Service call successful")
                break
            else:
                rospy.logwarn("Service call failed")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('change_angles')

    parser = argparse.ArgumentParser(description='Set model state in Gazebo')
    parser.add_argument('model_name', type=str, help='Name of the Gazebo model')
    parser.add_argument('--position', nargs='+', type=float, help='Position vector (x y z) or (x y)')
    parser.add_argument('--orientation', nargs='+', type=float, help='Orientation quaternion (x y z w) or Euler angles(r p y)')
    args = parser.parse_args()

    if not args.position and not args.orientation:
        rospy.loginfo("Please provide either --position or --orientation.")
        rospy.signal_shutdown('Error')

    call_set_model_state_service(args.model_name, position=args.position, orientation=args.orientation)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass