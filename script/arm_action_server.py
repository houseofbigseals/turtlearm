#!/usr/bin/env python
import rospy
# Brings in the SimpleActionClient
import actionlib
import control_msgs.msg
import sensor_msgs.msg
# Brings in the .action file and messages used by the move base action
from std_msgs.msg import String
#from control_msgs import *


def execute_cb_arm(goal):
    r = rospy.Rate(1)

    rospy.loginfo('===================DEBUG INFO==============')
    rospy.loginfo("arm goal recieved")
    print("list of all trajectory points from goal")
    print("each trajectory point is a list of positions")
    for i, point in enumerate(goal.trajectory.points):
        print(i, point.positions)
    rospy.loginfo('===================DEBUG INFO ENDS==============')
    # print(goal.trajectory.points[len(goal.trajectory.points)-1].positions)
    rospy.loginfo('===================POSE==============')
    goal_pose = sensor_msgs.msg.JointState()
    goal_pose.position = goal.trajectory.points[len(goal.trajectory.points)-1].positions
    rospy.loginfo("we have set final pose for arm topic as \n {}".format(goal_pose))
    #rospy.loginfo(goal_pose)
    arm_pub.publish(goal_pose)
    r.sleep()

    rospy.loginfo('===================Succeeded==============')
    tba_sas_arm.set_succeeded(result)
    rospy.loginfo(result)


def execute_cb_grip(goal):
    r = rospy.Rate(1)

    rospy.loginfo("gripper goal recieved")
    rospy.loginfo(goal)
    print(goal.trajectory.points[len(goal.trajectory.points)-1].positions)

    goal_pose = sensor_msgs.msg.JointState()
    rospy.loginfo('===================POSE==============')
    goal_pose.position = goal.trajectory.points[len(goal.trajectory.points)-1].positions
    rospy.loginfo(goal_pose)
    gripper_pub.publish(goal_pose)

    r.sleep()

    rospy.loginfo('===================Succeeded==============')
    tba_sas_grip.set_succeeded(result)
    rospy.loginfo(result)


rospy.init_node('tbarm_actions')
rospy.loginfo("Node started")

arm_pub = rospy.Publisher("arm/move_jp", sensor_msgs.msg.JointState, queue_size=10)
gripper_pub = rospy.Publisher("gripper/move_jp", sensor_msgs.msg.JointState, queue_size=10)


feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
tba_sas_arm = actionlib.SimpleActionServer("turtlearm/arm_controller/follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction, execute_cb=execute_cb_arm, auto_start = False)
tba_sas_grip = actionlib.SimpleActionServer("turtlearm/grip_controller/follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction, execute_cb=execute_cb_grip, auto_start = False)

tba_sas_arm.start()
rospy.loginfo("AS arm started")
tba_sas_grip.start()
rospy.loginfo("AS grip started")

result = control_msgs.msg.FollowJointTrajectoryResult()

rospy.loginfo("INIT done")

if __name__ == '__main__':
    rospy.loginfo("Waiting for SAC")
    rospy.spin()