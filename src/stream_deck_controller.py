#!/usr/bin/env python3

import rospy, os, yaml
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from PIL import Image, ImageDraw, ImageFont, ImageOps, ImageColor
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper

import sys
sys.path.append('/home/forest_ws/src/stream_deck_controller/src/utils/postprocess')
from manip_demo_rosbag_to_hdf5 import rosbag_to_hdf5

from std_msgs.msg import Bool

def read_config(config_path):
    """
    Reads a YAML configuration file and returns its contents as a dictionary.
    """
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

class StreamDeckButton:
    def __init__(self, position, label, background_color):
        self.position = position
        self.label = label
        self.background_color = background_color

    def change_background_color(self, color):
        self.background_color = color

    def get_position(self):
        return self.position

class StreamDeckController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('stream_deck_controller', anonymous=False)
        self.exit_loop = False
        self.rate = rospy.Rate(10)

        # Key styling parameters
        self.key_width = 100
        self.key_height = 100
        self.color_text = "#000000"
        self.background_color_active = "#27a007"
        self.background_color_inactive = "#bababa"

        self.recording_dataset = False
        self.policy_controlling = False
        self.last_bag_path = None
        self.replaying_bag = False


        # Publishers for deck actions
        self.home_publisher = rospy.Publisher('/streamdeck/home_position', Bool, queue_size=1)
        self.control_policy_publisher = rospy.Publisher('/streamdeck/control_policy', Bool, queue_size=1)

        # Service proxies for record and replay
        try:
            self.start_record_srv = rospy.ServiceProxy('/rosbag_recorder/start', Trigger)
            self.stop_record_srv  = rospy.ServiceProxy('/rosbag_recorder/stop',  Trigger)
            self.start_replay_srv = rospy.ServiceProxy('/rosbag_player/start',   Trigger)
            self.stop_replay_srv  = rospy.ServiceProxy('/rosbag_player/stop',    Trigger)
        except rospy.ServiceException as e:
            rospy.logerr(f"Could not connect to rosbag services: {e}")

        # Open Stream Deck
        try:
            self.stream_deck = DeviceManager().enumerate()[0]
            self.stream_deck.open()
            self.stream_deck.reset()
            self.stream_deck.set_brightness(100)
        except IndexError:
            rospy.logerr("No Stream Deck found.")
            quit()

        # Load fonts and background image
        self.assets_path =  "/home/forest_ws/src/stream_deck_controller/assets"
        self.font = ImageFont.truetype(os.path.join(self.assets_path, 'Roboto-Regular.ttf'), 14)
        self.background_image = Image.new("RGB", (self.key_width, self.key_height), color=ImageColor.getrgb("#000000"))

        # rosbag_directory from config
        cfg = read_config("/home/forest_ws/src/stream_deck_controller/config/config.yaml")
        self.bag_directory = cfg['general']['rosbag_directory']
        self.task_name     = cfg['general']['task_name']

        # Lay out keys
        self.column, self.row = self.stream_deck.key_layout()
        self.initialize_buttons()
        self.stream_deck.set_key_callback(self.on_key_change)
        rospy.on_shutdown(self.shutdown_callback)
        self.main_loop()

    def shutdown_callback(self):
        # Stop main loop and cleanup deck
        self.exit_loop = True

    def main_loop(self):
        while not self.exit_loop:
            self.rate.sleep()
        self.stream_deck.set_brightness(0)
        self.stream_deck.close()

    def on_key_change(self, deck, key, state):
        if not state:
            return

        # HOME POSITION button pressed
        if key == self.home_button.get_position():
            self.home_publisher.publish(True)
            self.home_button.change_background_color(self.background_color_active)

        # RECORD DATASET start/stop
        elif key == self.record_button.get_position():
            if not self.recording_dataset:
                rospy.loginfo("Starting dataset recording...")
                resp = self.start_record_srv(TriggerRequest())
                if resp.success:
                    self.recording_dataset = True
                    self.create_button(self.record_button, self.background_color_active)
                    # Extract path from service message
                    parts = resp.message.split(': ',1)
                    if len(parts)==2:
                        self.last_bag_path = parts[1].strip()
            else:
                rospy.loginfo("Stopping dataset recording...")
                resp = self.stop_record_srv(TriggerRequest())
                if resp.success:
                    self.recording_dataset = False
                    self.create_button(self.record_button, self.background_color_inactive)

        # REPLAY bag start/stop
        elif key == self.replay_button.get_position():
            if not self.replaying_bag and self.last_bag_path:
                rospy.loginfo("Starting bag replay...")
                resp = self.start_replay_srv(TriggerRequest())
                if resp.success:
                    self.replaying_bag = True
                    self.create_button(self.replay_button, self.background_color_active)
            elif self.replaying_bag:
                rospy.loginfo("Stopping bag replay...")
                resp = self.stop_replay_srv(TriggerRequest())
                if resp.success:
                    self.replaying_bag = False
                    self.create_button(self.replay_button, self.background_color_inactive)

        # DELETE LAST BAG
        elif key == self.delete_bag_button.get_position() and self.last_bag_path:
            os.remove(self.last_bag_path)
            rospy.loginfo(f"Deleted bag: {self.last_bag_path}")
            self.last_bag_path = None
            self.create_button(self.delete_bag_button, self.background_color_active)

        # ROSBAG TO HDF5
        elif key == self.convert_rosbag_to_hdf5_button.get_position():
            rospy.logwarn("Creating HDF5, please wait...")
            rosbag_path = os.path.join(self.bag_directory, self.task_name)
            if os.path.exists(rosbag_path):
                rosbag_to_hdf5(rosbag_path)
                rospy.loginfo("HDF5 created successfully.")
            self.create_button(self.convert_rosbag_to_hdf5_button, self.background_color_active)

        # POLICY CONTROL toggling
        elif key == self.policy_control_button.get_position():
            self.policy_controlling = not self.policy_controlling
            self.control_policy_publisher.publish(self.policy_controlling)
            color = self.background_color_active if self.policy_controlling else self.background_color_inactive
            self.create_button(self.policy_control_button, self.background_color_active if self.policy_controlling else self.background_color_inactive)

        # Reset inactive visuals for one-shot buttons
        for btn in [self.home_button, self.delete_bag_button, self.convert_rosbag_to_hdf5_button]:
            if key == btn:
                self.create_button(btn, btn.label, self.background_color_inactive)

        self.watchdog()

    def watchdog(self):
        # Placeholder for watchdog functionality
        pass

    def create_button(self, button, background_color):
        img = self.background_image.copy()
        draw = ImageDraw.Draw(img)
        # Draw background
        draw.rectangle([(0,0),(self.key_width,self.key_height)], fill=ImageColor.getrgb(background_color))
        # Center text
        draw.text((10,40), button.label, fill=ImageColor.getrgb(self.color_text), font=self.font)
        # Send to deck
        key_img = PILHelper.to_native_format(self.stream_deck, img)
        self.stream_deck.set_key_image(button.position, key_img)
        

    def initialize_buttons(self):
        self.home_button = StreamDeckButton(self._button_index(0, 0), "HOME POSITION", self.background_color_inactive)
        self.record_button = StreamDeckButton(self._button_index(0, 1), "RECORD DATASET", self.background_color_inactive)
        self.replay_button = StreamDeckButton(self._button_index(0, 2), "REPLAY BAG", self.background_color_inactive)
        self.delete_bag_button = StreamDeckButton(self._button_index(0, 3), "DELETE LAST BAG", self.background_color_inactive)
        self.convert_rosbag_to_hdf5_button = StreamDeckButton(self._button_index(0, 4), "ROSBAG TO HDF5", self.background_color_inactive)
        self.policy_control_button = StreamDeckButton(self._button_index(0, 5), "POLICY CONTROL", self.background_color_inactive)

        # Create initial visuals
        self.create_button(self.home_button, self.background_color_inactive)
        self.create_button(self.record_button, self.background_color_inactive)
        self.create_button(self.replay_button, self.background_color_inactive)
        self.create_button(self.delete_bag_button, self.background_color_inactive)
        self.create_button(self.convert_rosbag_to_hdf5_button, self.background_color_inactive)
        self.create_button(self.policy_control_button, self.background_color_inactive)

    def _button_index(self, col, row):
        return col * self.row + row

if __name__ == '__main__':
    StreamDeckController()
