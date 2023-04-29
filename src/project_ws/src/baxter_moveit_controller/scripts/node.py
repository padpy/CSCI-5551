import sys
import rospy
import moveit_commander
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import pi, tau, dist, fabs, cos
import copy

class BaxterMoveitController(object):
    def __init__(self):
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('moveit_baxter_example', anonymous=True)

        # Instantiate robot interface object
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")

    def run(self):
        # Initialize moveit_commander and rospy node

        self._home()

        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    def move(self, x, y, z, roll, pitch, yaw):
        current_pose = self.group.get_current_pose(end_effector_link='right_gripper').pose

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
        
        self.group.set_pose_target(target_pose, end_effector_link='right_gripper')

        self.group.plan()

        self.group.go(wait=True)
        
        #Ensures no residual movement
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        
    def Orient_Gripper(self):
        current_pose = self.group.get_current_pose(end_effector_link='right_gripper').pose
        target_pose = current_pose
        
        orientation = quaternion_from_euler(np.pi, 0, 0)
        target_pose.orientation.x = orientation[0]
        target_pose.orientation.y = orientation[1]
        target_pose.orientation.z = orientation[2]
        target_pose.orientation.w = orientation[3]
        
        self.group.set_pose_target(target_pose, end_effector_link='right_gripper')

        self.group.plan()

        self.group.go(wait=True)
        
        #Ensures no residual movement
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        
        #To Do: Grasp Object
        #Grasp Object: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        # grasping_group = 'right_gripper'
        # touch_links = self.robot.get_link_names(group=grasping_group)
        # scene.attach_box(eef_link, box_name, touch_links=touch_links)
        
    def smooth_move_to_initial(self):
        waypoints = []
        waypoints_e= []
        
        scale = 1.0
        
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        wpose.position.y -=  0.5  # First move up (z)
        wpose.position.z +=  0.8
        waypoints.append(copy.deepcopy(wpose))
        
        (plan, fraction) = self.group.compute_cartesian_path(
        waypoints, 0.01, 0.0)
        print("Executing plan")
        self.group.execute(plan, wait=True)
        
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets() 
        
        
        group_variable_values = self.group.get_current_joint_values()
        print("variable 1: " + str(group_variable_values[0]))
        group_variable_values[0] = group_variable_values[0] + 1.4
        self.group.set_joint_value_target(group_variable_values)

        plan2 = self.group.plan()
        
        self.group.go(wait = True)
        
        self.group.stop()
        
        print("Done!")
    
    #Move_to_block will pass in a block integer value 1-5 where block 1 is closest to Baxter and block 5 is farthest from Baxter.
    def move_to_block(self, block):
        print("Moving Block: " + str(block))
        waypoints = []
        
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        #wpose.position.x +=  0.7        # First move away from Baxter
        wpose.position.y += 0.09   
        wpose.position.z -= 0.10
        waypoints.append(copy.deepcopy(wpose))
        
        (plan, fraction) = self.group.compute_cartesian_path(
        waypoints, 0.01, 0.0)
        print("Executing plan")
        
        self.group.execute(plan, wait=True)
        
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        
        #Orient Gripper to pickup block
        self.Orient_Gripper()
        
        
        #Second portion move gripper to position just above our blocks. Intermediate position between initial position and block - 
        #helpful for calibration with simulated and physcial baxter. 
        waypoints2 = []
        
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        
        if(block == 1):
            wpose.position.x += -0.1542 
            wpose.position.y += -0.002426
        
        elif(block == 2):
            wpose.position.x += -0.074447
            
        elif(block == 3):
            wpose.position.x += 0.0
        
        elif(block == 4):
            wpose.position.x += 0.079342
            wpose.position.y += -0.007645 
            
        elif(block == 5):
        
            # group_variable_values = self.group.get_current_joint_values()
            # print("variable 1: " + str(group_variable_values[5]))
            # group_variable_values[5] = group_variable_values[5] - 0.3
            # self.group.set_joint_value_target(group_variable_values)
            # wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose

            # plan2 = self.group.plan()
            
            # self.group.go(wait = True)
            
            # self.group.stop()
            
            #print("Done!")
            wpose.position.x += 0.154084
            wpose.position.y += -0.005812
        
        waypoints2.append(copy.deepcopy(wpose))
        
        (plan2, fraction) = self.group.compute_cartesian_path(
        waypoints2, 0.01, 0.0)
        print("Executing plan")
        
        self.group.execute(plan2, wait=True)
        
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        
        
        value = input("Type something: ")
        
        #Third portion: Finally move to actual block position
        
        waypoints3 = []
        
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        
        wpose.position.z -= 0.08
        
        waypoints3.append(copy.deepcopy(wpose))
        
        (plan3, fraction) = self.group.compute_cartesian_path(
        waypoints3, 0.01, 0.0)
        print("Executing plan")
        
        self.group.execute(plan3, wait=True)
        
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
        
        #Get Base Joint Values
        #base_robot_pose = self.group.get_current_joint_values()
        
        #Move to initial State (uncomment)
        self.smooth_move_to_initial()
        
        initial_robot_pose = self.group.get_current_joint_values()
        
        self.move_to_block(3)
        
        my_input = "input"
        my_input = input("Type anything to continue: ")
        if(my_input is not None):
            #Move Back To Initial Position
            self.move_to_base_position(initial_robot_pose)
        
        
        
        #Move Back To Base Position
        #self.move_to_base_position(base_robot_pose)
        
        # my_input = "input"
        # while(my_input is not None):
            # my_input = [float(x) for x in input("Enter 6 value separated by a comma. First 3 values are position (x, y, z), last 3 are orientation (x, y, z).\n").split(', ')]
            # if(my_input is None):
                # break
            # else:
                # #print(self.shoulder.get_current_pose().pose)
                
                # self.move_joint(my_input[0], my_input[1])
            # #self.move(1.5, 0.0, -0.5, np.pi, 0, 0)
                # print("Moving back to initial position")
                # self.group = moveit_commander.MoveGroupCommander("left_arm")
                # self.move_to_base_position(current_robot_pose)
                # group_names = self.robot.get_group_names()
                
                # #print(self.group.get_current_joint_values("left_s0"))
                # print("============ Available Planning Groups:", self.robot.get_group_names())
                # print("============ Available Planning Joints:", self.group.get_joints())
                # self.group.stop()
                # self.group.clear_pose_targets() 
                

if __name__ == '__main__':
    try:
        BaxterMoveitController().run()
    except rospy.ROSInterruptException:
        pass