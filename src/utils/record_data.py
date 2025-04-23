#!/usr/bin/env python3

import os, re, subprocess, datetime, yaml

import rospy
from std_srvs.srv import Trigger, TriggerResponse

def read_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

class RosbagRecorder:
    def __init__(self):
        rospy.init_node('rosbag_recorder', anonymous=False)

        self.topics_to_record = read_config("/home/forest_ws/src/stream_deck_controller/config/config.yaml")['rosbag_topics']
        self.bag_directory = read_config("/home/forest_ws/src/stream_deck_controller/config/config.yaml")['general']['rosbag_directory']
        self.task_name = read_config("/home/forest_ws/src/stream_deck_controller/config/config.yaml")['general']['task_name']

        if not os.path.exists(self.bag_directory):
            os.makedirs(self.bag_directory)

        self.process = None

        rospy.Service('~start',  Trigger, self.start_recording)
        rospy.Service('~stop',   Trigger, self.stop_recording)

    def start_recording(self, req):
        if self.process is not None:
            return TriggerResponse(success=False, message="Already recording.")

        task_dir = os.path.join(self.bag_directory, self.task_name)
        if not os.path.exists(task_dir):
            os.makedirs(task_dir)

        bag_base_name = self._generate_next_bag_name(task_dir)
        output_path = os.path.join(task_dir, bag_base_name)

        command = ['rosbag', 'record', '-O', output_path] + self.topics_to_record
        self.process = subprocess.Popen(command)

        return TriggerResponse(success=True, message=f"Recording started: {output_path}.bag")

    def stop_recording(self, req):
        if self.process is None:
            return TriggerResponse(success=False, message="Not recording.")

        self.process.terminate()
        self.process.wait()
        self.process = None

        return TriggerResponse(success=True, message="Recording stopped.")

    def _generate_next_bag_name(self, task_dir):
        date_str = datetime.datetime.now().strftime("%Y%m%d")
        prefix  = f"{date_str}_"
        pattern = re.compile(rf"^{prefix}(\d+)\.bag$")

        max_index = 0
        for filename in os.listdir(task_dir):
            match = pattern.match(filename)
            if match:
                idx = int(match.group(1))
                if idx > max_index:
                    max_index = idx

        next_index = max_index + 1
        return f"{prefix}{next_index:05d}"

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    recorder = RosbagRecorder()
    recorder.spin()
