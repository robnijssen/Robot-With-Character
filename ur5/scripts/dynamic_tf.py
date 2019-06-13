#!/usr/bin/env python
import roslib
import rospy
import tf
import tf.msg
import geometry_msgs.msg
import math

class DynamicTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/dice", tf.msg.tfMessage, queue_size=10)

        change = 0.0
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "camera_vision_position"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "dice"
            t.transform.translation.x = 0.05 * math.sin(change)
            t.transform.translation.y = 0.05 * math.cos(change)
            t.transform.translation.z = -0.3

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1
            change += 0.1
            tfm = tf.msg.tfMessage([t])
            self.pub_tf.publish(tfm)
            
            

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    tfb = DynamicTFBroadcaster()
    rospy.spin()
