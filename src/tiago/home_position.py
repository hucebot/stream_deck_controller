#!/usr/bin/env python3

from cartesian_interface.pyci_all import *

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import os
import time


class HomePosition:
    def __init__(self):
        rospy.init_node("home_position", anonymous=False)

        # Init CartesI/O client
        self.client = pyci.CartesianInterfaceRos()
        self.postural_lambda = 0.0015
        self.timeout = 10
        self.error = 0.05

        self.home_posture = {
            "arm_left_1_joint": -1.05,
            "arm_left_2_joint": 0.61,
            "arm_left_3_joint": 2.74,
            "arm_left_4_joint": 1.78,
            "arm_left_5_joint": 1.90,
            "arm_left_6_joint": -0.68,
            "arm_left_7_joint": 0.16,

            "arm_right_1_joint": -1.05,
            "arm_right_2_joint": 0.61,
            "arm_right_3_joint": 2.74,
            "arm_right_4_joint": 1.78,
            "arm_right_5_joint": 1.90,
            "arm_right_6_joint": -0.68,
            "arm_right_7_joint": 0.16,

            "torso_lift_joint": 0.34,
        }

        self.cartesio_sol_topic = "/cartesian/solution"
        self.home_position_topic = "/streamdeck/home_position"

        rospy.loginfo("Home node started")

        rospy.Subscriber(self.home_position_topic, Bool, self.home_position_callback)
        self.home_position_publisher = rospy.Publisher('/streamdeck/reset_initial_state', Bool, queue_size=1)

    def home_position_callback(self, msg):
        if msg.data:
            rospy.logwarn("Moving to home position. Please don't move the robot")
            self.client.update()
            postural = self.client.getTask("Postural")
            lambda0 = postural.getLambda()

            postural.setReferencePosture(self.home_posture)
            postural.setLambda(self.postural_lambda)
            self.client.getTask("gripper_left_grasping_frame").disable()
            self.client.getTask("gripper_right_grasping_frame").disable()

            done = False
            start_time = time.time()
            while not done:
                time.sleep(1)
                msg = rospy.wait_for_message(
                    self.cartesio_sol_topic, JointState, timeout=1
                )
                for j in self.home_posture.keys():
                    if (
                        abs(self.home_posture[j] - msg.position[msg.name.index(j)])
                        > self.error
                    ):
                        done = False
                        break
                    else:
                        done = True
                if time.time() - start_time > self.timeout:
                    break
            
            rospy.logwarn("Home position reached. Now you can move the robot")
            postural.setLambda(lambda0)
            self.client.getTask("gripper_left_grasping_frame").enable()
            self.client.getTask("gripper_right_grasping_frame").enable()
            
            
            self.home_position_publisher.publish(Bool(data=True))


if __name__ == "__main__":
    try:
        HomePosition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
