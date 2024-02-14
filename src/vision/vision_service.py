#!/usr/bin/env python3

from __future__ import print_function
from ur5lego.srv import BlockPosition, BlockPositionResponse
from ur5lego.msg import Pose
import rospy

def handle_block_position(req):
    rospy.loginfo("Returning block position")
    msg = Pose()
    msg.position.x = 0.0
    msg.position.y = 0.0
    msg.position.z = 0.0
    msg.orientation.x = 0.0
    msg.orientation.y = 0.0
    msg.orientation.z = 0.0
    return BlockPositionResponse(msg)

def block_position_server():
    rospy.init_node("vision_service")
    s = rospy.Service("get_position", BlockPosition, handle_block_position)
    rospy.loginfo("Ready to return block position")
    rospy.spin()

if __name__ == "__main__":
    block_position_server()
