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


class BaxterIKController(object):
    def __init__(self):
        rospy.init_node("baxter_ik_controller")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        rs.enable()

        self._joint_accuracy = 0.3
        self._motion_timeout = 20.0
        self._arms = {
            "left": baxter_interface.Limb("left"),
            "right": baxter_interface.Limb("right"),
        }

        self._home_pose = {
            "left": self.cartestian_pose(0.5, 0.2, 0.15, 0.14034926870185907, 0.9897608157331103, 0.011245608178220349, 0.023433879552554254),
            "right": self.cartestian_pose(0.5, -0.2, 0.15, 0.0, 1.0, 0.0, 0.0),
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

        if response_seed[0] != response.RESULT_INVALID:
            rospy.logerr(f"Invalid path")
            # TODO: Throw exception

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
        self.go_to_pose(self._home_pose['left'], 'left')
        self.go_to_pose(self._home_pose['right'], 'right')


def main():
    controller = BaxterIKController()
    controller.run()


if __name__ == "__main__":
    main()
