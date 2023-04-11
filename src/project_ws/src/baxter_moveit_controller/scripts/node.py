import rospy
import moveit_commander
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

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

        self._home()

        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def _home(self):
        current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose

        orientation = quaternion_from_euler
        print(current_pose)

        target_pose = current_pose
        target_pose.position.x = 0.0
        target_pose.position.y = 0.75
        target_pose.position.z = 0.25

        orientation = quaternion_from_euler(np.pi, 0, 0)
        target_pose.orientation.x = orientation[0]
        target_pose.orientation.y = orientation[1]
        target_pose.orientation.z = orientation[2]
        target_pose.orientation.w = orientation[3]

        self.group.set_pose_target(target_pose, end_effector_link='left_gripper')

        self.group.plan()

        self.group.go(wait=True)

        target_pose.position.x = 0.75
        target_pose.position.y = 0.0
        target_pose.position.z = 0.25

        orientation = quaternion_from_euler(np.pi, 0, 0)
        target_pose.orientation.x = orientation[0]
        target_pose.orientation.y = orientation[1]
        target_pose.orientation.z = orientation[2]
        target_pose.orientation.w = orientation[3]

        self.group.set_pose_target(target_pose, end_effector_link='left_gripper')

        self.group.plan()

        self.group.go(wait=True)

if __name__ == '__main__':
    try:
        BaxterMoveitController().run()
    except rospy.ROSInterruptException:
        pass