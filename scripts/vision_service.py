#!/usr/bin/env python

from __future__ import print_function
from ur5lego.srv import BlockPosition, BlockPositionResponse
from ur5lego.msg import Pose
import rospy

def handle_block_position(req):
    print("Returning block position")
    msg = Pose()
    msg.position.x = 1
    msg.position.y = 2
    msg.position.z = 3
    msg.orientation.x = 0
    msg.orientation.y = 0
    msg.orientation.z = 0
    return BlockPositionResponse(msg)

def block_position_server():
    rospy.init_node("vision_service")
    s = rospy.Service("block_position", BlockPosition, handle_block_position)
    print("Ready to return block position")
    rospy.spin()

if __name__ == "__main__":
    block_position_server()