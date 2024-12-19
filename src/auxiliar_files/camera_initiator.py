#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Bool
import paramiko
import time


class CameraInitiator():
    def __init__(self):
        rospy.init_node('ros_camera_initiator', anonymous=False)

        rospy.loginfo('Camera Initiator node started.')

        rospy.Subscriber('/streamdeck/camera_stream', Bool, self.start_ros_control_camera)

        self.ip = rospy.get_param('~ip_computer', '192.168.50.76')
        self.username = rospy.get_param('~username', 'gstreamer')
        self.password = rospy.get_param('~password', 'gstreamer')
        self.ssh_client = None
        self.PID_process = None
        self.shell = None

        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def initialize_camera(self):
        try:
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh_client.connect(self.ip, username=self.username, password=self.password)
            self.shell = self.ssh_client.invoke_shell()
        except Exception as e:
            rospy.logerr(f'Error during SSH connection. Check connection to the jetson nano: {e}')
            self.ssh_client = None

        self.shell.send('docker rm -f ros2_streamer\n')
        try:
            time.sleep(2)
            self.shell.send("sh run_gstreamer.sh\n")
            time.sleep(1)
            rospy.loginfo("ROS2 camera started")
        except Exception as e:
            rospy.logerr(f'Error during SSH connection or command execution: {e}')
            self.ssh_client = None

    def start_ros_control_camera(self, msg):
        if msg.data:
            rospy.loginfo("Starting ROS2 camera service...")
            self.initialize_camera()

        else:
            rospy.loginfo("Stopping ROS2 camera service...")
            if self.ssh_client is not None:
                self.ssh_client.close()
                self.ssh_client = None
                rospy.loginfo("ROS2 camera service stopped.")

    def shutdown(self):
        rospy.loginfo("Shutting down camera Initiator node...")
        if self.ssh_client is not None:
            rospy.loginfo("Closing SSH connection...")
            self.shell.send("docker rm -f ros2_streamer\n")
            self.ssh_client.close()
            self.ssh_client = None
        rospy.loginfo("Camera Initiator node stopped.")

if __name__ == '__main__':
    try:
        CameraInitiator()
    except rospy.ROSInterruptException:
        pass
