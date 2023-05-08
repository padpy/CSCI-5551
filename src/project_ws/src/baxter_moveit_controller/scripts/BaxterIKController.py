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


class BaxterIKController(object):
    def __init__(self):
        rospy.init_node("baxter_ik_controller")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        rs.enable()

        #Fiducial Listener
        self.Fiducial_Listener = tf.TransformListener()

        sleep(5)
        transform_array = []
        transform_array = np.zeros(3)
        samples = 0
        while samples != 10:
            (transform, _rotation) = self.Fiducial_Listener.lookupTransform('/reference/base', 'fiducial_0', rospy.Time(0))
            samples = samples + 1
            transform_array[0] += transform_array[0]; transform_array[1] += transform_array[1]; transform_array[2] += transform_array[2]

        #x = transform_array[0]/10; y = transform_array[1]/10; z = transform_array[2]/10

        self._joint_accuracy = 0.3
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
            "left": self.cartestian_pose( 0.5,  0.4, 0.2, 0.0, 1.0, 0.0, 0.0),
            "right": self.cartestian_pose(0.5, -0.2, 0.2, 0.0, 1.0, 0.0, 0.0),
        }

        self._to_fiducial = {
            "right": self.cartestian_pose(0.45,  0.03, -0.23, -0.35, 1.0, 0.0, 0.0),
        }

        self._block_3 = {
            "right": self.cartestian_pose(0.54, -0.01, -0.23, 0.0, 1.0, 0.0, 0.0), #.09, -.04, -0.23
        }

        self._block_2 = {
            "right": self.cartestian_pose(0.67,  -0.05, -0.23, 0.0, 1.0, 0.0, 0.0)
        }

        self._pickup_block_3 = {
            "right": self.cartestian_pose(0.54,  -0.01, -0.36, 0.0, 1.0, 0.0, 0.0)
        }

        self._place_block_5 = {
            "right": self.cartestian_pose(0.68,  -0.01, -0.28, 0.0, 1.0, 0.0, 0.0)
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

    def go_to_pose(self, pose, limb):
        self._arms[limb].move_to_joint_positions(
            self.joints_from_pose(pose, limb),
            self._motion_timeout,
            self._joint_accuracy
        )

    def _service_name(self, limb):
        return f"ExternalTools/{limb}/PositionKinematicsNode/IKService"

    def run(self):
        #Fiducial Cartesian Hard-Coded
        x_delta = 0.58; y_delta = 0.03; z_delta = -0.2

        #self.go_to_pose(self._home_pose['left'], 'left')
        self.go_to_pose(self._home_pose['right'], 'right')

        #self.go_to_pose(self._to_fiducial['right'], 'right')
        #self._arms['right']
        #self._gripper['right'].open()
        #self._gripper['right'].close()


        self.go_to_pose(self._block_3['right'], 'right')
        rospy.sleep(3)
        self.go_to_pose(self._pickup_block_3['right'], 'right')
        rospy.sleep(1)
        self._gripper['right'].close()
        self.go_to_pose(self._home_pose['right'], 'right')
        rospy.sleep(1)
        self.go_to_pose(self._place_block_5['right'], 'right')
        rospy.sleep(1)
        self._gripper['right'].open()
        rospy.sleep(1)
        self.go_to_pose(self._home_pose['right'], 'right')



def main():
    controller = BaxterIKController()
    controller.run()


if __name__ == "__main__":
    main()
