import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def publish_static_transform():
    rospy.init_node('static_transform_publisher')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = 'world'
    transform.child_frame_id = 'child_frame'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0

    broadcaster.sendTransform([transform])
    rospy.spin()

if __name__ == '__main__':
    publish_static_transform()
