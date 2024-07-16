#!/usr/bin/env python 
import rospy
from tf2_msgs.msg import TFMessage

# Fkn bodge this is!
if __name__ == '__main__':
    rospy.init_node('transforms_passthru')

    publisher = rospy.Publisher('tf', TFMessage, queue_size=1)
    def callback(msg):
        publisher.publish(msg)

    listener = rospy.Subscriber("tf_remapped", TFMessage, callback)

    rospy.spin()
    

    