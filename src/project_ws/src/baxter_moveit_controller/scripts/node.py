import rospy
import moveit_commander
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
import numpy as np
from copy import deepcopy

class BaxterMoveitController(object):
    def __init__(self):
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('moveit_baxter_example', anonymous=True)

        # Instantiate robot interface object
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("left_arm")


    def run(self):
        # Initialize moveit_commander and rospy node

        # self.home()
        self.draw_box()

        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def home(self):
        current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose

        # target_pose = self._pose_from_rpy(current_pose, 0.0, 0.75, 0.25, np.pi, 0, 0)
        # self._execute_plan(target_pose)

        target_pose = self._pose_from_rpy(current_pose, 0.75, 0.0, 0.25, np.pi, 0, 0)
        self._execute_plan(target_pose)


    def draw_box(self):
        current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose

        target_pose = self._pose_from_rpy(current_pose, 0.85, 0.0, 0, np.pi, 0, 0)
        self._execute_plan(target_pose)

        target_pose = self._pose_from_rpy(current_pose, 0.85, 0.1, 0.35, np.pi, 0, 0)
        self._execute_plan(target_pose)

        target_pose = self._pose_from_rpy(current_pose, 0.75, 0.1, 0.35, np.pi, 0, 0)
        self._execute_plan(target_pose)

        target_pose = self._pose_from_rpy(current_pose, 0.75, 0.0, 0.25, np.pi, 0, 0)
        self._execute_plan(target_pose)


    def _pose_from_rpy(self, pose, x, y, z, roll, pitch, yaw):
        pose = deepcopy(pose)
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        orientation = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        return pose


    def _execute_plan(self, pose):
        self.group.set_pose_target(pose, end_effector_link='left_gripper')
        self.group.plan()
        self.group.go(wait=True)

if __name__ == '__main__':
    try:
        BaxterMoveitController().run()
    except rospy.ROSInterruptException:
        pass