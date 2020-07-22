import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from joybro.msg import JoyBro

class ArmTeleop(object):
    """
    xArm teleoperation via joybro device
    """

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_teleop_arm')

        # params of teleoperation
        self.joysub = rospy.Subscriber('/joybro', JoyBro, self.joy_callback)
        # btn threshold
        self.threshold = rospy.get_param('~threshold', 20)
        self.dx = 0.05
        self.dy = 0.05
        self.dz = 0.05


        self.new_goal = geometry_msgs.msg.Pose()

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: {}".format(self.eef_link))

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        # go to loop
        self.loop()

    def loop(self):
        rospy.spin()

    def joy_callback(self, joy_msg):
        #
        current_pose = self.move_group.get_current_pose().pose
        # joy_msg.
        new_goal = geometry_msgs.msg.Pose()
        state_updated_flag = False

        if abs(joy_msg.right_x) > self.threshold:
            # new delta to self.pose.x will be proportional to the msg.x field, that we got
            new_goal.position.x = current_pose.position.x + (joy_msg.right_x/512.0)*self.dx
            print("new_goal.position.x : {}".format(new_goal.position.x))

        if abs(joy_msg.right_y) > self.threshold:
            # new delta to self.pose.x will be proportional to the msg.x field, that we got
            new_goal.position.y = current_pose.position.y + (joy_msg.right_y/512.0)*self.dy
            print("new_goal.position.y : {}".format(new_goal.position.y))

        if abs(joy_msg.left_y) > self.threshold:
            # new delta to self.pose.x will be proportional to the msg.x field, that we got
            new_goal.position.z = current_pose.position.z + (joy_msg.left_y/512.0)*self.dz
            print("new_goal.position.z : {}".format(new_goal.position.z))

        # check if smth new was arrived
        if current_pose != new_goal:
            print("something changed in pose, try to plan it")
            # plan moving to new pose
            # set target
            self.move_group.set_pose_target(new_goal)

            ## Now, we call the planner to compute the plan and execute it.
            print("============== executed plan")
            plan = self.move_group.go(wait=True)
            print(plan)
            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.move_group.clear_pose_targets()

