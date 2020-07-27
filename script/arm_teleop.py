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

sign = lambda x: x and (1, -1)[x < 0]


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
        self.threshold = rospy.get_param('~threshold', 50)
        self.dx = 0.1
        self.dy = 0.1
        self.dz = 0.1


        # self.new_goal = geometry_msgs.msg.Pose()
        self.state = "wait"

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=1)

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

        self.new_goal = self.move_group.get_current_pose().pose


        self.rate = rospy.Rate(2)
        # go to loop
        self.loop()

    def loop(self):
        try:
            while not rospy.core.is_shutdown():
                if self.state == "moving":
                    print("in moving state")
                    self.move_to_point(self.new_goal)
                    print("return to state wait")
                    self.state = "wait"
                self.rate.sleep()
        except KeyboardInterrupt:
            print("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')

    def joy_callback(self, joy_msg):
        #
        if self.state == "wait":
            # if we got msg in moving state, we just fucked it up
            current_pose = self.move_group.get_current_pose().pose
            new_goal = copy.deepcopy(current_pose)  # it is important to use deepcopy

            if abs(joy_msg.right_x) > self.threshold:
                # new delta to self.pose.x will be proportional to the msg.x field, that we got
                new_goal.position.x = current_pose.position.x + (sign(joy_msg.right_x))*self.dx
                print("new_goal.position.x : {}".format(new_goal.position.x))

            if abs(joy_msg.right_y) > self.threshold:
                # new delta to self.pose.x will be proportional to the msg.x field, that we got
                new_goal.position.y = current_pose.position.y + (sign(joy_msg.right_y))*self.dy
                print("new_goal.position.y : {}".format(new_goal.position.y))

            if abs(joy_msg.left_y) > self.threshold:
                # new delta to self.pose.x will be proportional to the msg.x field, that we got
                new_goal.position.z = current_pose.position.z + (sign(joy_msg.left_y))*self.dz
                print("new_goal.position.z : {}".format(new_goal.position.z))

            # check if smth new was arrived
            if current_pose != new_goal:
                print("==== current pose: {}".format(current_pose))
                print("==== new pose: {}".format(new_goal))
                self.new_goal = new_goal
                self.state = "moving"

    def move_to_point(self, pose):
        print("something changed in pose, try to plan it")
        # plan moving to new pose
        # set target
        self.move_group.set_pose_target(pose)

        ## Now, we call the planner to compute the plan and execute it.
        print("============== executed plan")
        plan = self.move_group.go(wait=True)
        print(plan)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def move_cartesian(self, pose):
        print("new pose is: {}".format(pose))

        waypoints = [pose]
        print("============== executed plan")
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        print(plan)

        self.move_group.execute(plan, wait=True)


if __name__ == "__main__":
    ArmTeleop()