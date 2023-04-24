import sys
import rospy
import moveit_commander
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import pi, tau, dist, fabs, cos

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
        
    def move(self, x, y, z, roll, pitch, yaw):
        current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose

        orientation = quaternion_from_euler
        print("Current Pose: " + str(current_pose))

        target_pose = current_pose
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        orientation = quaternion_from_euler(roll, pitch, yaw)
        target_pose.orientation.x = orientation[0]
        target_pose.orientation.y = orientation[1]
        target_pose.orientation.z = orientation[2]
        target_pose.orientation.w = orientation[3]
        
        self.group.set_pose_target(target_pose, end_effector_link='left_gripper')

        self.group.plan()

        self.group.go(wait=True)
        
        #Ensures no residual movement
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        
    def move_to_base_position(self, base_coordinate):
        
        #orientation = quaternion_from_euler
        #orientation = quaternion_from_euler(np.pi, 0, 0)
        
        #orientation = quaternion_from_euler(roll, pitch, yaw)
        #target_pose.orientation.x = orientation[0]
        #target_pose.orientation.y = orientation[1]
        #target_pose.orientation.z = orientation[2]
        #target_pose.orientation.w = orientation[3]
        
        target_pose = base_coordinate
    
        self.group.set_joint_value_target(base_coordinate)
        
        self.group.plan()

        success = self.group.go(wait=True)
        
        #Ensures no residual movement
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()   

        print("Returned to starting position: " + str(success))

    def _home(self):
        #current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose
        
        #Move to initial State
        self.move(0.0, 0.75, 0.25, np.pi, 0, 0)
        self.move(0.75, 0.0, 0.25, np.pi, 0, 0)
        
        #Grab joint values for initial state
        current_robot_pose = self.group.get_current_joint_values()
        
        my_input = 0.0
        
        while(my_input is not None):
            my_input = [float(x) for x in input("Enter 6 value separated by a comma. First 3 values are position (x, y, z), last 3 are orientation (x, y, z).\n").split(', ')]
            if(my_input is None):
                break
            else:
                self.move(my_input[0], my_input[1], my_input[2], my_input[3], my_input[4], my_input[5])
            #self.move(1.5, 0.0, -0.5, np.pi, 0, 0)
                print("Moving back to initial position")
                self.move_to_base_position(current_robot_pose)


if __name__ == '__main__':
    try:
        BaxterMoveitController().run()
    except rospy.ROSInterruptException:
        pass