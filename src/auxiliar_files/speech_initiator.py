#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Bool
import paramiko
import time


class SpeechInitiator():
    def __init__(self):
        rospy.init_node('speech_initiator', anonymous=False)

        rospy.loginfo('Speech Initiator node started.')

        rospy.Subscriber('/streamdeck/speech', Bool, self.start_speech_cb)

        self.ip = rospy.get_param('~ip_computer', '192.168.50.162')
        self.username = rospy.get_param('~username', 'pal')
        self.password = rospy.get_param('~password', 'pal')
        self.ssh_client = None
        self.PID_process = None
        self.shell = None

        try:
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh_client.connect(self.ip, username=self.username, password=self.password)
            self.shell = self.ssh_client.invoke_shell()
            time.sleep(1)
        except Exception as e:
            rospy.logerr(f'Error during SSH connection. Check connection to jetson nano: {e}')
            self.ssh_client = None

        rospy.on_shutdown(self.shutdown)
        rospy.spin()


    def initialize_speech(self):
        try:
            self.shell.send('python3 start_speech.py\n')
            time.sleep(1)
            rospy.loginfo("Speech script started")

        except Exception as e:
            rospy.logerr(f'Error running speech script: {e}')

    def start_speech_cb(self, msg):
        if msg.data:
            rospy.loginfo("Starting speech node...")
            self.initialize_speech()

        else:
            rospy.loginfo("Stopping speech node...")
            if self.ssh_client is not None:
                self.ssh_client.close()
                self.ssh_client = None

    def shutdown(self):
        rospy.loginfo("Shutting down Speech Initiator node...")
        if self.ssh_client is not None:
            rospy.loginfo("Closing SSH connection...")
            self.shell.send('exit\n')
            self.ssh_client.close()
            self.ssh_client = None

if __name__ == '__main__':
    try:
        SpeechInitiator()
    except rospy.ROSInterruptException:
        pass
