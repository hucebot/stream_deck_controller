#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Bool
import paramiko
import time


class EmergencyStop():
    def __init__(self):
        rospy.init_node('emergency_stop', anonymous=False)

        rospy.loginfo('Emergency Stop node started.')

        rospy.Subscriber('/streamdeck/emergency_stop', Bool, self.emergency_stop_callback)
        rospy.spin()

    def emergency_stop_callback(self, msg):
        if msg.data:
            rospy.loginfo("Emergency stop activated.")
            os.system("rosnode kill /ros_control_bridge")
            rospy.loginfo("ROS control bridge stopped.")
            #TODO: Add more nodes to stop and shutdown the motors
        else:
            rospy.loginfo("Emergency stop deactivated.")

if __name__ == '__main__':
    try:
        EmergencyStop()
    except rospy.ROSInterruptException:
        pass