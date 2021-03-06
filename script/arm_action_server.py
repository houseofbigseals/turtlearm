#!/usr/bin/env python
import rospy
# Brings in the SimpleActionClient
import actionlib
import control_msgs.msg
import sensor_msgs.msg
# Brings in the .action file and messages used by the move base action
from std_msgs.msg import String
#from control_msgs import *


class ArmController(object):

    def __init__(self):
        self.deviation = 0.5
        rospy.init_node('tbarm_actions')
        rospy.loginfo("Node started")

        self.arm_pub = rospy.Publisher("/arm/move_jp", sensor_msgs.msg.JointState, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper/move_jp", sensor_msgs.msg.JointState, queue_size=10)

        rospy.Subscriber("/arm/measured_js", sensor_msgs.msg.JointState, self.arm_js_cb)
        rospy.Subscriber("/gripper/measured_js", sensor_msgs.msg.JointState, self.gripper_js_cb)


        self.feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
        self.tba_sas_arm = actionlib.SimpleActionServer("turtlearm/arm_controller/follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb_arm, auto_start = False)
        self.tba_sas_grip = actionlib.SimpleActionServer("turtlearm/grip_controller/follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb_grip, auto_start = False)

        self.joints_in_deviation = list()

        self.tba_sas_arm.start()
        rospy.loginfo("AS arm started")
        self.tba_sas_grip.start()
        rospy.loginfo("AS grip started")
        self.result = control_msgs.msg.FollowJointTrajectoryResult()
        rospy.loginfo("INIT done")


    def arm_js_cb(self, arm_js):
        self.arm_js = arm_js

    def gripper_js_cb(self, gripper_js):
        self.gripper_js = gripper_js

    def execute_cb_arm(self, goal):
        r = rospy.Rate(50)

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
        for i in range(len(goal.trajectory.points)):
            #i =+1
            # rospy.loginfo('===goal.trajectory.points.positions[i] {}'.format(goal.trajectory.points[i]))
            for k in range(len(goal.trajectory.points[i].positions)):
                self.joints_in_deviation.append(False)
            goal_pose.position = goal.trajectory.points[i].positions
            rospy.loginfo("we have set {} pose for arm topic as \n".format(i))
            self.arm_pub.publish(goal_pose)
            rospy.loginfo('published goal pose : \n {}'.format(goal_pose))
            while not all(self.joints_in_deviation):
                for j in range(len(goal.trajectory.points[i].positions)):
                    # j =+1
                    if abs(goal.trajectory.points[i].positions[j] - self.arm_js.position[j]) < self.deviation:
                        self.joints_in_deviation[j] = True
                    else:
                        self.joints_in_deviation[j] = False
                r.sleep()
            self.joints_in_deviation = list()

        rospy.loginfo('===================Succeeded==============')
        self.tba_sas_arm.set_succeeded(self.result)
        rospy.loginfo(self.result)


    def execute_cb_grip(self, goal):
        r = rospy.Rate(50)

        rospy.loginfo("gripper goal recieved")
        rospy.loginfo(goal)
        print(goal.trajectory.points[len(goal.trajectory.points)-1].positions)

        goal_pose = sensor_msgs.msg.JointState()
        rospy.loginfo('===================POSE==============')
        goal_pose.position = goal.trajectory.points[len(goal.trajectory.points)-1].positions
        rospy.loginfo(goal_pose)
        self.gripper_pub.publish(goal_pose)

        r.sleep()

        rospy.loginfo('===================Succeeded==============')
        self.tba_sas_grip.set_succeeded(self.result)
        rospy.loginfo(self.result)


if __name__ == '__main__':
    ArmController()
    rospy.loginfo("Waiting for SAC")
    rospy.spin()