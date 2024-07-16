#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
import geometry_msgs.msg
import tf
from tf import transformations as t
import moveit_commander
import moveit_msgs.msg
import sys
import tf2_ros
import numpy as np

possible_frames = ["tag_0", "tag_1"]
possible_states = ["pick", "down", "forward", "right_up", "forward_right", "down_left"]



def main():
    state_counter = 0
    last = 0
    # Initialise node
    rospy.init_node('apriltag_controller')
    rospy.sleep(2)

    # Initialise moveit commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Start commander class and planning scene interface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    rospy.sleep(2)

    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id="arm_base_link"
    plane_pose.header.stamp = rospy.Time.now()
    plane_pose.pose.position.z = -0.12
    scene.add_box("floor", plane_pose, size=(2,1,0.1))

    # And tf listener
    listener = tf.TransformListener()
    gripper_pub = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)

    while not rospy.is_shutdown():
        for target in possible_frames:
            try:
                now = rospy.Time.now()
                listener.waitForTransform("webcam", target, now, rospy.Duration(1.0)) # Prevents queue building up!
                (trans, rot) = listener.lookupTransform("webcam", target, now) # Get the transform
                if target == possible_frames[0]:
                    print("Tag 0 recognised, opening")
                    gripper_pub.publish(0.5)
                else:
                    print("Tag 1 recognised, closing")
                    gripper_pub.publish(1.2) 
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                #print(str(e))
                continue
            
            print(str([trans[0], trans[1]]))
            if trans[0] < 0.1 and trans[1] < 0.1 and last != 0:
                state_counter -= 1
                last = 0
            elif trans[0] > 0.1 and trans[1] > 0.1 and last != 1:
                state_counter += 1
                last = 1
            else:
                last=2
            
            if state_counter < 0:
                state_counter = 0
            if state_counter > len(possible_states)-1:
                state_counter = len(possible_states)-1
            group.set_named_target(possible_states[state_counter])
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()


          
           


if __name__ == '__main__':
    main()