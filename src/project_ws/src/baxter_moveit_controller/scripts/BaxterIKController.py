#!/usr/bin/env python3

import struct
import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import tf
import numpy as np
from time import sleep
from baxter_moveit_controller.srv import t3

class BaxterIKController(object):
    def __init__(self):
        rospy.init_node("baxter_ik_controller")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        rs.enable()

        #Fiducial Listener
        self.Fiducial_Listener = tf.TransformListener()

        self._joint_accuracy = 0.03
        self._motion_timeout = 20.0
        self._arms = {
            "left": baxter_interface.Limb("left"),
            "right": baxter_interface.Limb("right"),
        }
        self._gripper = {
            "left": baxter_interface.Gripper("left"),
            "right": baxter_interface.Gripper("right"),
        }

        self._gripper["right"].calibrate()

        self._home_pose = {
            "left": self.cartestian_pose( 0.5,  0.4, 0.2,    0.0,   1.0,   0.0, 0.0),
            "right": self.cartestian_pose(0.6, -0.1, 0.0,   -0.707, 0.707, 0.0, 0.0),
        }

        self._initialization = {
            "left": self.cartestian_pose(0.1,   0.4, 0.4, 0.0, 1.0, 0.0, 0.0),
            "right": self.cartestian_pose(0.1, -0.4, 0.4, 0.0, 1.0, 0.0, 0.0)
        }

        self._fiducial_loc = [0,0,0]
        self._fiducial_offset = [-0.03, -0.0762, -0.0462]
        self._location_offsets = {
            'block1': [0.08,0.075,0],
            'block2': [0.08,0.0375,0],
            'block3': [0.08,0,0],
            'block4': [0.08,-0.0375,0],
            'block5': [0.075,-0.075,0],
            'board1': [0.23,0.04,0],
            'board2': [0.23,0,0],
            'board3': [0.23,-0.04,0],
            'board4': [0.19,0.04,0],
            'board5': [0.19,0,0],
            'board6': [0.19,-0.04,0],
            'board7': [0.15,0.04,0],
            'board8': [0.15,0,0],
            'board9': [0.15,-0.04,0],
        }

        self._control_rate = rospy.Rate(20.0)

        self.ik_service_left = rospy.ServiceProxy(
            self._service_name("left"), SolvePositionIK
        )
        rospy.wait_for_service(self._service_name("left"), 5.0)

        self.ik_service_right = rospy.ServiceProxy(
            self._service_name("right"), SolvePositionIK
        )
        rospy.wait_for_service(self._service_name("right"), 5.0)


    def cartestian_pose(self, x, y, z, qx, qy, qz, qw):
        return Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )


    def joints_from_pose(self, pose, limb):
        pose_stamped = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'), pose=pose
        )

        service = self.ik_service_left if limb == "left" else self.ik_service_right
        try:
            request = SolvePositionIKRequest()
            request.pose_stamp.append(pose_stamped)
            response = service(request)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"Service call failed: {e}")
            # TODO: Throw exception

        response_seed = struct.unpack(
            "<%dB" % len(response.result_type), response.result_type
        )

        if response_seed[0] == response.RESULT_INVALID:
            err_str = f"Invalid path"
            rospy.logerr(err_str)
            raise RuntimeError(err_str)

        return dict(zip(response.joints[0].name, response.joints[0].position))

    def go_to_location(self, loc, limb, speed=0.05):
        self.go_to_pose(
            self.cartestian_pose(
                self._fiducial_loc[0] + self._location_offsets[loc][0],
                self._fiducial_loc[1] + self._location_offsets[loc][1],
                self._fiducial_loc[2] + self._location_offsets[loc][2],
                -0.707, 0.707, 0.0, 0.0
            ),
            limb,
            speed
        )

    def go_to_pose(self, pose, limb, speed=0.05):
        self._arms[limb].set_joint_position_speed(speed)
        self._arms[limb].move_to_joint_positions(
            self.joints_from_pose(pose, limb),
            self._motion_timeout,
            self._joint_accuracy
        )

    def _service_name(self, limb):
        return f"ExternalTools/{limb}/PositionKinematicsNode/IKService"

    def get_next_move_client(board):
        rospy.wait_for_service('get_next_move')
        try:
            get_next_move = rospy.ServiceProxy('get_next_move', t3)
            resp1 = get_next_move(board)
            return resp1.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def set_fiducial_loc(self, n_samples=1):
        sleep(1)
        transform_array = np.zeros(3)
        samples = 0
        while samples != n_samples:
            (transform, _) = self.Fiducial_Listener.lookupTransform('/base', 'fiducial_0', rospy.Time(0))
            samples += 1
            transform_array[0] += transform[0]; transform_array[1] += transform[1]; transform_array[2] += transform[2]

        x = transform_array[0]/n_samples; y = transform_array[1]/n_samples; z = transform_array[2]/n_samples
        self._fiducial_loc = [x + self._fiducial_offset[0],
                              y + self._fiducial_offset[1],
                              z + self._fiducial_offset[2]]
        print(f'Fiducial Coordinates: {self._fiducial_loc}')

    def run(self):
        #Fiducial Cartesian Hard-Coded
        x_delta = 0.58; y_delta = 0.03; z_delta = -0.2

        self.go_to_pose(self._home_pose['left'], 'left')
        self.go_to_pose(self._home_pose['right'], 'right')

        self.set_fiducial_loc()

        self.go_to_location('block1', 'right')
        sleep(0.25)
        self._gripper['right'].close()
        sleep(0.25)
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)
        self.go_to_location('board1', 'right')
        sleep(0.25)
        self._gripper['right'].open()
        sleep(0.25)
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)

        self.go_to_location('block2', 'right')
        sleep(0.25)
        self._gripper['right'].close()
        sleep(0.25)
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)
        self.go_to_location('board3', 'right')
        sleep(0.25)
        self._gripper['right'].open()
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)

        self.go_to_location('block3', 'right')
        sleep(0.25)
        self._gripper['right'].close()
        sleep(0.25)
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)
        self.go_to_location('board5', 'right')
        sleep(0.25)
        self._gripper['right'].open()
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)

        self.go_to_location('block4', 'right')
        sleep(0.25)
        self._gripper['right'].close()
        sleep(0.25)
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)
        self.go_to_location('board7', 'right')
        sleep(0.25)
        self._gripper['right'].open()
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)

        self.go_to_location('block5', 'right')
        sleep(0.25)
        self._gripper['right'].close()
        sleep(0.25)
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)
        self.go_to_location('board9', 'right')
        sleep(0.25)
        self._gripper['right'].open()
        self.go_to_pose(self._home_pose['right'], 'right')
        sleep(0.25)

def main():
    controller = BaxterIKController()
    controller.run()


if __name__ == "__main__":
    main()
