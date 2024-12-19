#!/usr/bin/env python3

from cartesian_interface.pyci_all import *
from cartesian_interface.srv import SetTransform, SetTransformRequest

import rospy
import tf2_ros
from std_msgs.msg import Bool

import os


class ResetOdometry:
    def __init__(self):
        rospy.init_node("reset_odometry", anonymous=False)

        # Set up tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Init CartesI/O client
        self.client = pyci.CartesianInterfaceRos()

        # Initi CartesI/O service proxy
        self.srv_proxy = rospy.ServiceProxy("/cartesian/reset_base", SetTransform)

        # Wait for tf listener
        rospy.sleep(1)

        rospy.loginfo("Emergency Stop node started.")

        rospy.Subscriber(
            "/streamdeck/reset_odometry", Bool, self.reset_odometry_callback
        )

    def reset_odometry_callback(self, msg):
        if msg.data:
            self.client.update()
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Velocity)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )

            req = SetTransformRequest()
            t = self.tf_buffer.lookup_transform("odom", "base_footprint", rospy.Time())
            req.pose.position.x = t.transform.translation.x
            req.pose.position.y = t.transform.translation.y
            req.pose.position.z = t.transform.translation.z
            req.pose.orientation.x = t.transform.rotation.x
            req.pose.orientation.y = t.transform.rotation.y
            req.pose.orientation.z = t.transform.rotation.z
            req.pose.orientation.w = t.transform.rotation.w
            self.srv_proxy(req)

            self.client.getTask("base_link").setControlMode(pyci.ControlType.Position)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
        else:
            pass


if __name__ == "__main__":
    try:
        ResetOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
