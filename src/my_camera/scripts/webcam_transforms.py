#!/usr/bin/env python 
import rospy
import math
import tf
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
from tf import transformations as t

possible_frames = ["Robot"]

if __name__ == '__main__':
    rospy.init_node('webcam_transformer')

    listener = tf.TransformListener()
    publisher = rospy.Publisher('tf_remapped', TFMessage, queue_size=1)

    while not rospy.is_shutdown():
        for target in possible_frames:
            try:
                (trans, rot) = listener.lookupTransform("webcam", target, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                #print(str(e))
                continue


            if target == "Robot":              
                # Invert it
                transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                inversed_transform = t.inverse_matrix(transform)

                translation = t.translation_from_matrix(inversed_transform)
                quaternion = t.quaternion_from_matrix(inversed_transform)
                frame_id = "Robot"
                child_frame_id = "webcam"
            
            else:
                # Copy it
                translation = trans
                quaternion = rot
                frame_id = "webcam"
                child_frame_id = target

            
            cmd = TFMessage()
            cmd.transforms.append(geometry_msgs.msg.TransformStamped())
            cmd.transforms[0].header.stamp = rospy.Time.now()
            cmd.transforms[0].header.frame_id = frame_id
            cmd.transforms[0].child_frame_id = child_frame_id
            cmd.transforms[0].transform.translation.x = translation[0]
            cmd.transforms[0].transform.translation.y = translation[1]
            cmd.transforms[0].transform.translation.z = translation[2]
            cmd.transforms[0].transform.rotation.x = quaternion[0]
            cmd.transforms[0].transform.rotation.y = quaternion[1]
            cmd.transforms[0].transform.rotation.z = quaternion[2]
            cmd.transforms[0].transform.rotation.w = quaternion[3]
            

            publisher.publish(cmd)#(translation, quaternion, rospy.Time.now(), target, "/webcam")
            possible_frames = listener.getFrameStrings()
            #print(listener.getFrameStrings())


        
