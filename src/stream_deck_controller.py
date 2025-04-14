#!/usr/bin/env python3

import rospy
import os
from std_srvs.srv import Trigger, TriggerRequest
from PIL import Image, ImageDraw, ImageFont, ImageOps, ImageColor
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper

class StreamDeckController:
    def __init__(self):
        rospy.init_node('stream_deck_controller', anonymous=False)
        self.exit_loop = False
        self.rate = rospy.Rate(10)

        self.key_width = 100
        self.key_height = 100
        self.color_text = "#000000"
        self.background_color_active = "#27a007"
        self.background_color_inactive = "#bababa"

        self.recording_dataset = False
        self.last_bag_path = None

        try:
            self.start_srv = rospy.ServiceProxy('/rosbag_recorder/start', Trigger)
            self.stop_srv = rospy.ServiceProxy('/rosbag_recorder/stop', Trigger)
        except rospy.ServiceException:
            rospy.logerr("Could not connect to rosbag services.")

        try:
            self.stream_deck = DeviceManager().enumerate()[0]
            self.stream_deck.open()
            self.stream_deck.reset()
            self.stream_deck.set_brightness(100)
        except IndexError:
            rospy.logerr("No Stream Deck found.")
            quit()

        # Load fonts/images
        self.assets_path = os.path.join(os.path.dirname(__file__), '../assets')
        self.font = ImageFont.truetype(os.path.join(self.assets_path, 'Roboto-Regular.ttf'), 14)
        self.background_image = Image.new("RGB", (self.key_width, self.key_height), color=ImageColor.getrgb("#000000"))
        
        self.column, self.row = self.stream_deck.key_layout()

        self.initialize_buttons()
        self.stream_deck.set_key_callback(self.on_key_change)
        rospy.on_shutdown(self.shutdown_callback)
        self.main_loop()

    def shutdown_callback(self):
        self.exit_loop = True

    def main_loop(self):
        while not self.exit_loop:
            self.rate.sleep()
        self.stream_deck.set_brightness(0)
        self.stream_deck.close()

    def on_key_change(self, deck, key, state):
        if state:
            # HOME POSITION
            if key == self.home_position_button:
                self.create_button(self.home_position_button, "HOME POSITION", self.background_color_active)

            # RECORD DATASET
            elif key == self.record_position_button and not self.recording_dataset:
                try:
                    resp = self.start_srv(TriggerRequest())
                    if resp.success:
                        self.recording_dataset = True
                        self.create_button(self.record_position_button, "RECORD DATASET", self.background_color_active)

                        msg_parts = resp.message.split(': ', 1)
                        if len(msg_parts) == 2:
                            bag_path = msg_parts[1].strip()
                            self.last_bag_path = bag_path
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            elif key == self.record_position_button and self.recording_dataset:
                try:
                    resp = self.stop_srv(TriggerRequest())
                    if resp.success:
                        self.recording_dataset = False
                        self.create_button(self.record_position_button, "RECORD DATASET", self.background_color_inactive)
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            # DELETE LAST BAG
            elif key == self.delete_bag_button:
                self.create_button(self.delete_bag_button, "DELETE LAST BAG", self.background_color_active)
                if self.last_bag_path:
                    try:
                        os.remove(self.last_bag_path)
                        rospy.loginfo(f"Deleted bag file: {self.last_bag_path}")
                        self.last_bag_path = None
                    except Exception as e:
                        rospy.logerr(f"Could not delete bag file: {e}")
        else:
            if key == self.home_position_button:
                self.create_button(self.home_position_button, "HOME POSITION", self.background_color_inactive)
            elif key == self.delete_bag_button:
                self.create_button(self.delete_bag_button, "DELETE LAST BAG", self.background_color_inactive)
            self.watchdog()

    def watchdog(self):
        pass

    def create_button(self, position, label, background_color):
        image = self.background_image.copy()
        label = label.upper()
        x_pos = 0
        y_pos = 40

        if " " not in label:
            x_pos = 50 - (len(label) * 5)
        else:
            list_words = label.split(" ")
            label = ""
            for word in list_words:
                len_word = len(word)
                if len_word > 7:
                    label += " "*4 + word + "\n"
                elif len_word > 5:
                    label += " "*7 + word + "\n"
                else:
                    label += " "*10 + word + "\n"

        draw = ImageDraw.Draw(image)
        draw.rectangle([(0, 0), (self.key_width, self.key_height)], fill=ImageColor.getrgb(background_color))

        if "EMERGENCY" in label:
            self.color_text = "#ffffff"
        else:
            self.color_text = "#000000"

        draw.text((x_pos, y_pos), label, fill=ImageColor.getrgb(self.color_text), font=self.font)
        
        key_image = PILHelper.to_native_format(self.stream_deck, image)
        self.stream_deck.set_key_image(position, key_image)
        return position

    def initialize_buttons(self):
        # HOME BUTTON
        self.home_position_button = (0, 0)
        self.home_position_button = self.home_position_button[0] * self.row + self.home_position_button[1]
        self.create_button(self.home_position_button, "HOME POSITION", self.background_color_inactive)

        # RECORD BUTTON
        self.record_position_button = (0, 3)
        self.record_position_button = self.record_position_button[0] * self.row + self.record_position_button[1]
        self.create_button(self.record_position_button, "RECORD DATASET", self.background_color_inactive)

        # DELETE BAG BUTTON
        self.delete_bag_button = (0, 4)
        self.delete_bag_button = self.delete_bag_button[0] * self.row + self.delete_bag_button[1]
        self.create_button(self.delete_bag_button, "DELETE LAST BAG", self.background_color_inactive)

if __name__ == '__main__':
    StreamDeckController()
