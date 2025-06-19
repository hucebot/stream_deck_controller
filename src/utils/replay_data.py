#!/usr/bin/env python3

import os
import subprocess
import rospy
from std_srvs.srv import Trigger, TriggerResponse

class RosbagPlayer:
    def __init__(self):
        rospy.init_node('rosbag_player', anonymous=False)

        self.bag_path = rospy.get_param('~bag_path', '/home/forest_ws/src/stream_deck_controller/rosbags/telepresence_experiment/telepresence_experiment.bag')

        self.process = None

        rospy.Service('/rosbag_player/start', Trigger, self.start_playback)
        rospy.Service('/rosbag_player/stop',  Trigger, self.stop_playback)

        rospy.loginfo("rosbag_player node ready. Services available: start, stop.")

    def start_playback(self, req):
        rospy.loginfo("Received request to start playback.")
        if self.process is not None and self.process.poll() is None:
            return TriggerResponse(success=False,
                                   message="Rosbag is already playing.")

        if not os.path.isfile(self.bag_path):
            return TriggerResponse(success=False,
                                   message=f"Bag file not found: {self.bag_path}")
        cmd = [
            'rosbag', 'play', self.bag_path,
            '--quiet',
            '/gripper_right_grasping_frame/read:=/dxl_input/pos_right'
        ]

        self.process = subprocess.Popen(cmd)
        rospy.loginfo(f"Starting rosbag play: {' '.join(cmd)}")
        return TriggerResponse(success=True,
                               message=f"Playback started: {os.path.basename(self.bag_path)}")

    def stop_playback(self, req):
        if self.process is None or self.process.poll() is not None:
            return TriggerResponse(success=False,
                                   message="No active playback to stop.")

        self.process.terminate()
        self.process.wait()
        rospy.loginfo("Playback stopped.")
        self.process = None
        return TriggerResponse(success=True,
                               message="Playback stopped.")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    player = RosbagPlayer()
    player.spin()
