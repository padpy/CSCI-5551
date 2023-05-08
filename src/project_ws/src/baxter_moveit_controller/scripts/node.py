import sys
import rospy
import moveit_commander
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import pi, tau, dist, fabs, cos
import copy
import baxter_interface
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Pose
from std_msgs.msg import String


class BaxterMoveitController(object):
    def __init__(self):
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('moveit_baxter_example', anonymous=True)
        
        #Fiducial listener
        self.Fiducial_Listener = tf.TransformListener()

        # Instantiate robot interface object
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.right.calibrate()

        #Instantiate next move
        self.move = ""
        self.previous_move = ""

    #Tic-Tac-Toe Subscriber in order to get next move.   
    def callback(self, data):
        self.set_move(data.data)

    def listener(self):
        #rospy.init_node('listener', anonymous = True)
        rospy.Subscriber("chatter", String, self.callback)
        rospy.sleep(1)

    def set_move(self, data):
        self.previous_move = self.move
        self.move = data

    def get_fidicual_displacement(self):
        transform_array = []
        transform_array = np.zeros(3)
        samples = 0
        rate = rospy.Rate(10.0)
        while samples != 10:
            (transform, rotation) = self.Fiducial_Listener.lookupTransform('/base', 'fiducial_0', rospy.Time(0))
            samples = samples + 1
            transform_array[0] += transform[0]; transform_array[1] += transform[1]; transform_array[2] += transform[2];
                
        transform_array[0] = transform_array[0]/10; transform_array[1] = transform_array[1]/10; transform_array[2] = transform_array[2]/10;
        x = transform[0]; y = transform[1]; z = transform[2];
        return x, y, z

    def run(self):
        # Initialize moveit_commander and rospy node
        self.listener()
        block_count = 1
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))

        self._home()
        initial_robot_pose = self.group.get_current_joint_values()
        x, y, z = self.get_fidicual_displacement()

        while not rospy.is_shutdown():
            if(self.previous_move != self.move):
                self.move_to_block(x, y, z, block_count)
                self.move_to_base_position
                self.Place_Block(x, y, z, self.move)
                self.move_to_base_position
                block_count = block_count + 1
            
            #Testing purposes
            '''
            my_input = "input"
            my_input = input("Type anything to continue: ")
            '''
            message = input("Enter a command (type 'exit' to quit): ")
            if message == "exit":
                break
            s.sendall(message.encode())
            #my_input = s.recv(1024).decode() #Uncomment when server's all ready to go
            print(f"Received {my_input}")

            if(my_input is not None):
                #Move Back To Initial Position
                self.move_to_base_position(initial_robot_pose)

            # When finished shut down moveit_commander.
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)
    
    def move(self, x, y, z, roll, pitch, yaw):
        current_pose = self.group.get_current_pose(end_effector_link='right_gripper').pose

        orientation = quaternion_from_euler
        print("Current Pose: " + str(current_pose))

        target_pose = current_pose
        target_pose.position.x = x + 0.1
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

        self.rotate_wrist(-0.9)

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
        waypoints.append(self.make_pose(0.2, -0.9, 0.12, wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))

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
        joint_vals = self.group.get_current_joint_values()
        print("Joint Values: " + str(joint_vals[-1]))

        print("Done!")

    def make_pose(self, x, y, z, qx, qy, qz, qw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

    #Move_to_block will pass in a block integer value 1-5 where block 1 is closest to Baxter and block 5 is farthest from Baxter.
    def move_to_block(self, x, y, z, block):
        print("Moving Block: " + str(block))
        waypoints = []

        self.right.open()

        x = x + 0.07
        z = z + 0.1 #Hover above block
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        waypoints.append(self.make_pose(x, y, z, wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))
        #waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
        waypoints, 0.01, 0.0)
        print("Executing plan")

        self.group.execute(plan, wait=True)

        self.group.stop()

        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        rospy.sleep(1)

        #Orient Gripper to pickup block
        #self.Orient_Gripper()
        self.rotate_wrist(-1.0)


        #Second portion move gripper to position just above our blocks. Intermediate position between initial position and block -
        #helpful for calibration with simulated and physcial baxter.
        waypoints2 = []

        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose

        if(block == 1):
            wpose.position.y -= 0.08

        elif(block == 2):
            wpose.position.y += -0.04 

        elif(block == 3):
            print("")

        elif(block == 4):
            wpose.position.y += 0.04

        elif(block == 5):
            wpose.position.y += 0.08

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

        wpose.position.z -= 0.05

        waypoints3.append(copy.deepcopy(wpose))

        (plan3, fraction) = self.group.compute_cartesian_path(
        waypoints3, 0.01, 0.0)
        print("Executing plan")

        self.group.execute(plan3, wait=True)

        self.group.stop()

        #Clear targets after planning poses.
        self.group.clear_pose_targets()

        #Manipulating objects requires the robot be able to touch them without the planning scene reporting the contact as a collision.
        #By adding link names to the touch_links array, we are telling the planning scene to ignore collisions between those links and the box.
        self.right.close(True)
        self.right.set_moving_force(50)
        self.right.set_holding_force(50)
        #Possible MoveIt implementation
        #grasping_group = 'right_gripper'
        #touch_links = self.robot.get_link_names(group=grasping_group)
        #scene.attach_box(self.group.get_end_effector_link(), "block_1_0_clone_0", touch_links=touch_links)
    
    def Place_Block(self, x, y, z, position):
        waypoints = []

        self.right.close()

        x = x + 0.19
        z = z + 0.2 #Hover above block
        y = y + 0.001
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        waypoints.append(self.make_pose(x, y, z, wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w))
        #waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
        waypoints, 0.01, 0.0)
        print("Executing plan")

        self.group.execute(plan, wait=True)

        self.group.stop()

        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        
        #Move to position
        waypoints_position = []
        
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        
        if(position == 1):
            wpose.position.x += 0.055
            wpose.position.y += 0.055
        elif(position == 2):
            wpose.position.x += 0.1
        elif(position == 3):
            wpose.position.x += 0.055
            wpose.position.y += -0.055
        elif(position == 4):
            wpose.position.y += 0.055
        elif(position == 6):
            wpose.position.y += -0.055
            
        waypoints_position.append(copy.deepcopy(wpose))

        (plan_position, fraction) = self.group.compute_cartesian_path(
        waypoints_position, 0.01, 0.0)
        print("Executing plan")

        self.group.execute(plan_position, wait=True)

        self.group.stop()

        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        rospy.sleep(4)

        waypoints3 = []
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        wpose.position.z -= 0.14
        waypoints3.append(copy.deepcopy(wpose))
        
        (plan3, fraction) = self.group.compute_cartesian_path(
        waypoints3, 0.01, 0.0)
        print("Executing plan")

        self.group.execute(plan3, wait=True)
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()

        #self.right.open()

        rospy.sleep(5)

        waypoints4 = []
        wpose = self.group.get_current_pose(end_effector_link = 'right_gripper').pose
        wpose.position.z += 0.10
        waypoints4.append(copy.deepcopy(wpose))
        
        (plan4, fraction) = self.group.compute_cartesian_path(
        waypoints4, 0.01, 0.0)
        print("Executing plan")

        self.group.execute(plan4, wait=True)
        self.group.stop()
        
        #Clear targets after planning poses.
        self.group.clear_pose_targets()
        rospy.sleep(3)
    
    def rotate_wrist(self, angle):
        # TODO: This can throw an error moveit_commander.exception.MoveItCommanderException, due to motion being out of bounds.
        # I am not sure where in code this is thrown, so for right now I am going to leave this todo until we can find deteremine
        # the joint value bounds for the wrist either through code or experimentally.
        joint_vals = self.group.get_current_joint_values()
        print("Joint Values: " + str(joint_vals[-1]))
        joint_vals[-1] = angle
        self.group.set_joint_value_target(joint_vals)
        self.group.go()
        
        #Ensures no residual movement
        self.group.stop()

        #Clear targets after planning poses.
        self.group.clear_pose_targets()

    def move_to_base_position(self, base_coordinate):

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

        self.right.open()

        #Move to initial State (uncomment)
        self.smooth_move_to_initial()
        
        

if __name__ == '__main__':
    try:
        BaxterMoveitController().run()
    except rospy.ROSInterruptException:
        pass
