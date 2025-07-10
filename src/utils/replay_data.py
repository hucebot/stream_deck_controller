#!/usr/bin/env python3

import os, subprocess, yaml

import rospy
from std_srvs.srv import Trigger, TriggerResponse

def read_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config


class RosbagPlayer:
    def __init__(self):
        rospy.init_node('rosbag_player', anonymous=False)

        self.bag_directory = read_config("/home/forest_ws/src/stream_deck_controller/config/config.yaml")['general']['rosbag_directory']
        self.task_name = read_config("/home/forest_ws/src/stream_deck_controller/config/config.yaml")['general']['task_name']
        self.bag_path = os.path.join(self.bag_directory, self.task_name)

        self.process = None

        rospy.Service('/rosbag_player/start', Trigger, self.start_playback)
        rospy.Service('/rosbag_player/stop',  Trigger, self.stop_playback)

        rospy.loginfo("rosbag_player node ready. Services available: start, stop.")

    def get_last_bag_file(self, task_dir):
        bag_files = [f for f in os.listdir(task_dir) if f.endswith('.bag')]
        if not bag_files:
            rospy.logwarn(f"No bag files found in directory: {task_dir}")
            return None
        bag_files.sort()
        return os.path.join(task_dir, bag_files[-1])

    def start_playback(self, req):
        rospy.loginfo("Received request to start playback.")
        self.bag_path = self.get_last_bag_file(self.bag_path)
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
