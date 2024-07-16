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

possible_frames = ["tag_4"]
w1 = 1
w2 = -2


def main():
    jg = 0.0
    jgi = -0.1
    # Initialise moveit commander and ros node 
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('apriltag_controller')

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
    #scene.attach_box("arm_base_link", "floor", plane_pose, size=(2,1,0.1))

    # And tf listener
    listener = tf.TransformListener()
    print("Planning frame: " + str(group.get_planning_frame()))
    while not rospy.is_shutdown():
        for target in possible_frames:
            try:
                now = rospy.Time.now()
                listener.waitForTransform("webcam", target, now, rospy.Duration(4.0)) # Prevents queue building up!
                (trans, rot) = listener.lookupTransform("webcam", target, now) # Get the transform
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                #print(str(e))
                continue

            
            # pose_goal = geometry_msgs.msg.Pose()
            
            # Don't bother sending a command if we're close to the centre (of webcam view)
            # TODO shift to relative rather than absolute pose
            dist = np.linalg.norm([trans[0]*w1, trans[1]*w1])
            # if dist < 0.01:
            #     continue

            current_pose = group.get_current_pose()
            pose = current_pose

            # Position as 'impulse' from current pose
            pose.pose.position.x = 0.0# -0.1 #pose_goal.position.x #0.1 0.03 0.435
            pose.pose.position.y += trans[0]*w1# -0.03 
            pose.pose.position.z += trans[1]*w2# -0.435

            attached_objects = scene.get_attached_objects(["floor"])
            is_attached = len(attached_objects.keys()) > 0
        
            # Rotation dictated by Apriltag angle
            rot_new = t.quaternion_from_euler(0, t.euler_from_quaternion(rot)[2], 0)
            print("Translation : " + str([trans[0]*w1, trans[1]*w2]))
            print("Rotation : " + str(np.degrees(t.euler_from_quaternion(rot)[2])) +" degrees")
            print("Requested pose: \n" + str(pose.pose.position))
            #print("Floor?: " + str(is_attached))

            # Send to moveit
            group.set_pose_target(pose, end_effector_link="gripper_link")
            group.set_goal_position_tolerance(1.2) #1.2
            group.set_goal_orientation_tolerance(np.radians(30))
            #print("Tolerances: " +str(group.get_goal_tolerance()))

            group.set_planning_time(3.0)
            
            # Plan and go
            plan = group.plan()
            group.execute(plan, wait=True)

            # Calling `stop()` ensures that there is no residual movement
            group.stop()
            
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            group.clear_pose_targets()

            
            """ joint_goal = group.get_current_joint_values()
            joint_goal[0] = jg
            jg += jgi
            if jg < -1.4:
                jgi = 0.1
            elif jg > 1.4:
                jgi = -0.1
                
            group.go(joint_goal, wait=True)
            group.stop() """
           


if __name__ == '__main__':
    main()