#!/usr/bin/env python3

import rospy
import os
import threading
import paramiko
import time
import subprocess
import docker
import signal

from PIL import Image, ImageDraw, ImageFont, ImageColor
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper

from open_dashboard_robot import open_dashboard_robot
from open_dashboard_video import open_dashboard_video
from open_dashboard_wifi import open_dashboard_wifi
from open_dxl_input import open_dxl_input

from std_msgs.msg import Bool

class StreamDeckController:
    def __init__(self):
        rospy.init_node('stream_deck_controller', anonymous=False)
        self.exit_loop = False

        self.rate = rospy.Rate(10)

        self.config_file = rospy.get_param("~config_file", "cdonoso_config.yaml")
        self.layout_file = rospy.get_param("~layout_file", "cdonoso_layout.yaml")

        self.tiago_process_id = []
        self.base_station_process_id = []

        self.camera_active = False
        self.camera_error = False

        self.arm_teleoperation_active = False
        self.arm_control_error = False

        self.home_position_active = False
        self.home_position_error = False

        self.reset_odometry_active = False
        self.reset_odometry_error = False

        self.emergency_stop = False
        self.emergency_stop_blinking = False
        self.blinking_thread = None

        self.robot_dashboard_active = False
        self.robot_dashboard_error = False

        self.wifi_dashboard_active = False
        self.wifi_dashboard_error = False

        self.video_dashboard_active = False
        self.video_dashboard_error = False

        self.speech_active = False
        self.speech_error = False

        self.processes = []
        self.ports = [5000, 5001, 5002]

        self.record_active = False

        self.robot_dashboard_shell = None
        self.robot_dashboard_docker = docker.from_env()
        self.wifi_dashboard_shell = None
        self.wifi_dashboard_docker = docker.from_env()
        self.video_dashboard_shell = None
        self.video_dashboard_docker = docker.from_env()
        self.open_dxl_input_shell = None
        self.open_dxl_input_docker = docker.from_env()

        self.key_width = 100
        self.key_height = 100

        self.background_color = "#bababa"
        self.color_text = "#000000"
        self.background_color_active = "#27a007"
        self.background_color_inactive = "#bf1616"

        try:
            self.stream_deck = DeviceManager().enumerate()[0]
            self.stream_deck.open()
            self.stream_deck.reset()
            self.stream_deck.set_brightness(100)
        except IndexError:
            rospy.logerr("No Stream Deck found. Check the connection and try again.")
            self.exit_loop = True

        self.asset_path = "/home/forest_ws/src/stream_deck_controller/assets"#os.path.join(os.path.dirname(__file__), '../assets')
        print(self.asset_path)
        self.font = ImageFont.truetype(os.path.join(self.asset_path, 'Roboto-Regular.ttf'), 14)
        self.background_image = Image.new("RGB", (self.key_width, self.key_height), color=ImageColor.getrgb("#000000"))

        self.column, self.row = self.stream_deck.key_layout()

        self.initialize_buttons()
        self.stream_deck.set_key_callback(self.on_key_change)

        self.teleoperation_mode_publisher = rospy.Publisher('/streamdeck/teleoperation_mode', Bool, queue_size=10)
        self.camera_initiator_publisher = rospy.Publisher('/streamdeck/camera_stream', Bool, queue_size=10)
        self.microphone_publisher = rospy.Publisher('/streamdeck/microphone', Bool, queue_size=10)
        self.record_publisher = rospy.Publisher('/streamdeck/record', Bool, queue_size=10)
        self.emergency_stop_publisher = rospy.Publisher('/streamdeck/emergency_stop', Bool, queue_size=10)
        self.home_position_publisher = rospy.Publisher('/streamdeck/home_position', Bool, queue_size=10)
        self.reset_odometry_publisher = rospy.Publisher('/streamdeck/reset_odometry', Bool, queue_size=10)
        self.speech_initiator_publisher = rospy.Publisher('/streamdeck/speech', Bool, queue_size=10)

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
            if key == self.arm_teleoperation_button_position:
                self.handle_arm_teleoperation_button()

            elif key == self.camera_button_position:
                self.handle_camera_button()

            elif key == self.home_position_button_position:
                self.handle_home_position_button()

            elif key == self.reset_odometry_button_position:
                self.handle_reset_odometry_button()

            elif key == self.emergency_stop_button_position:
                self.handle_emergency_stop_button()

            elif key == self.speaker_button_position:
                self.handle_speaker_button()

            elif key == self.robot_dashboard_button_position:
                self.handle_robot_dashboard_button()

            elif key == self.wifi_dashboard_button_position:
                self.handle_wifi_dashboard_button()

            elif key == self.video_dashboard_button_position:
                self.handle_video_dashboard_button()

            elif key == self.record_button_position:
                self.handle_record_button()

            elif key == self.speech_button_position:
                self.handle_speech_button()

        else:
            self.speaker_button = self.create_button_img(self.speaker_button_position, "Speaker", os.path.join(self.asset_path, 'mic_default.png'))
            self.microphone_publisher.publish(False)
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


    def create_button_img(self, position, label, img_path):
        image = Image.open(img_path).convert('RGB')
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


        key_image = PILHelper.to_native_format(self.stream_deck, image)
        self.stream_deck.set_key_image(position, key_image)
        return position

    def handle_speech_button(self):
        self.speech_active = not self.speech_active
        if self.speech_active:
            self.speech_button = self.create_button_img(self.speech_button_position, "speech", os.path.join(self.asset_path, 'speech_active.png'))
            self.speech_initiator_publisher.publish(True)
        else:
            self.speech_button = self.create_button_img(self.speech_button_position, "speech", os.path.join(self.asset_path, 'speech_default.png'))
            self.speech_initiator_publisher.publish(False)


    def handle_arm_teleoperation_button(self):
        self.arm_teleoperation_active = not self.arm_teleoperation_active
        if self.arm_teleoperation_active and not self.emergency_stop_blinking:
            self.arm_teleoperation_button = self.create_button_img(self.arm_teleoperation_button_position, "WBD", os.path.join(self.asset_path, 'teleoperation_active.png'))
            self.teleoperation_mode_publisher.publish(True)
        else:
            self.arm_teleoperation_button = self.create_button_img(self.arm_teleoperation_button_position, "WBD", os.path.join(self.asset_path, 'teleoperation_default.png'))
            self.teleoperation_mode_publisher.publish(False)

    def handle_camera_button(self):
        self.camera_active = not self.camera_active
        if self.camera_active:
            self.camera_button = self.create_button_img(self.camera_button_position, "Camera", os.path.join(self.asset_path, 'camera_active.png'))
            self.camera_initiator_publisher.publish(True)
        else:
            self.camera_button = self.create_button_img(self.camera_button_position, "Camera", os.path.join(self.asset_path, 'camera_default.png'))
            self.camera_initiator_publisher.publish(False)

    def handle_home_position_button(self):
        self.home_position_active = not self.home_position_active
        if self.home_position_active and not self.emergency_stop_blinking:
            self.home_position_button = self.create_button_img(self.home_position_button_position, "Home", os.path.join(self.asset_path, 'home_active.png'))
            self.home_position_publisher.publish(True)
        else:
            self.home_position_button = self.create_button_img(self.home_position_button_position, "Home", os.path.join(self.asset_path, 'home_default.png'))
            self.home_position_publisher.publish(False)

    def handle_reset_odometry_button(self):
        self.reset_odometry_active = not self.reset_odometry_active
        if self.reset_odometry_active and not self.emergency_stop_blinking:
            self.reset_odometry_button = self.create_button_img(self.reset_odometry_button_position, "Odometry", os.path.join(self.asset_path, 'reset_odometry_active.png'))
            self.reset_odometry_publisher.publish(True)
        else:
            self.reset_odometry_button = self.create_button_img(self.reset_odometry_button_position, "Odometry", os.path.join(self.asset_path, 'reset_odometry_default.png'))
            self.reset_odometry_publisher.publish(False)


    def handle_emergency_stop_button(self):
        self.emergency_stop = not self.emergency_stop
        if self.emergency_stop:
            self.emergency_stop_blinking = True
            self.blinking_thread = threading.Thread(target=self.blink_emergency_stop)
            self.blinking_thread.start()
            self.arm_teleoperation_button = self.create_button(self.arm_teleoperation_button_position, "Enable Control", self.background_color)
            self.teleoperation_active = False
            self.arm_teleoperation_active = False
            self.emergency_stop_publisher.publish(True)

        else:
            self.emergency_stop_blinking = False
            if self.blinking_thread:
                self.blinking_thread.join()
            self.emergency_stop_button = self.create_button(self.emergency_stop_button_position, "Emergency Stop", self.background_color_inactive)
            self.emergency_stop_publisher.publish(False)

    def blink_emergency_stop(self):
        colors = [self.background_color_inactive, "#000000"]
        color_index = 0
        while self.emergency_stop_blinking:
            current_color = colors[color_index % 2]
            self.emergency_stop_button = self.create_button(self.emergency_stop_button_position, "Emergency Stop", current_color)
            color_index += 1
            time.sleep(0.2)

    def handle_speaker_button(self):
        self.speaker_button = self.create_button_img(self.speaker_button_position, "Speaker", os.path.join(self.asset_path, 'mic_active.png'))
        self.microphone_publisher.publish(True)


    def handle_robot_dashboard_button(self):
        self.robot_dashboard_active = not self.robot_dashboard_active
        if self.robot_dashboard_active:
            self.robot_dashboard_button = self.create_button_img(self.robot_dashboard_button_position, "RBD", os.path.join(self.asset_path, 'robot_dashboard_active.png'))
            rospy.loginfo("Launching robot dashboard")
            self.robot_dashboard_shell = open_dashboard_robot(self.robot_dashboard_shell, self.robot_dashboard_docker, config_file=self.config_file, layout_file=self.layout_file)

        else:
            self.robot_dashboard_button = self.create_button_img(self.robot_dashboard_button_position, "RBD", os.path.join(self.asset_path, 'robot_dashboard_default.png'))
            if self.robot_dashboard_shell:
                existing_container = self.robot_dashboard_docker.containers.get('robot_dashboard')
                existing_container.stop()
                existing_container.remove()
                rospy.loginfo("Killed and removed robot dashboard container")
                self.robot_dashboard_shell = None


    def handle_wifi_dashboard_button(self):
        self.wifi_dashboard_active = not self.wifi_dashboard_active
        if self.wifi_dashboard_active:
            self.wifi_dashboard_button = self.create_button_img(self.wifi_dashboard_button_position, "WBD", os.path.join(self.asset_path, 'wifi_dashboard_active.png'))
            rospy.loginfo("Launching Wi-Fi dashboard")
            self.wifi_dashboard_shell = open_dashboard_wifi(self.wifi_dashboard_shell, self.wifi_dashboard_docker, config_file=self.config_file, layout_file=self.layout_file)

        else:
            self.wifi_dashboard_button = self.create_button_img(self.wifi_dashboard_button_position, "WBD", os.path.join(self.asset_path, 'wifi_dashboard_default.png'))
            if self.wifi_dashboard_shell:
                existing_container = self.wifi_dashboard_docker.containers.get('wifi_dashboard')
                existing_container.stop()
                existing_container.remove()
                rospy.loginfo("Killed and removed Wi-Fi dashboard container")
                self.wifi_dashboard_shell = None


    def handle_video_dashboard_button(self):
        self.video_dashboard_active = not self.video_dashboard_active
        if self.video_dashboard_active:
            self.video_dashboard_button = self.create_button_img(self.video_dashboard_button_position, "RBD", os.path.join(self.asset_path, 'video_dashboard_active.png'))
            rospy.loginfo("Launching video dashboard")
            self.video_dashboard_shell = open_dashboard_video(self.video_dashboard_shell, self.video_dashboard_docker, config_file=self.config_file, layout_file=self.layout_file)

        else:
            self.video_dashboard_button = self.create_button_img(self.video_dashboard_button_position, "RBD", os.path.join(self.asset_path, 'video_dashboard_default.png'))
            if self.video_dashboard_shell:
                existing_container = self.video_dashboard_docker.containers.get('video_dashboard')
                existing_container.stop()
                existing_container.remove()
                rospy.loginfo("Killed and removed video dashboard container")
                self.video_dashboard_shell = None

    def handle_record_button(self):
        self.record_active = not self.record_active
        if self.record_active:
            self.record_button = self.create_button_img(self.record_button_position, "Record", os.path.join(self.asset_path, 'record_active.png'))
            self.record_publisher.publish(True)
        else:
            self.record_button = self.create_button_img(self.record_button_position, "Record", os.path.join(self.asset_path, 'record_default.png'))
            self.record_publisher.publish(False)


    def initialize_buttons(self):
        #### Robot Dashboard Button
        self.robot_dashboard_button_position = (0, 0)
        self.robot_dashboard_button_position = self.robot_dashboard_button_position[0] * self.row + self.robot_dashboard_button_position[1]
        self.robot_dashboard_button = self.create_button_img(self.robot_dashboard_button_position, "RBD", os.path.join(self.asset_path, 'robot_dashboard_default.png'))

        #### Wifi Dashboard Button
        self.wifi_dashboard_button_position = (0, 1)
        self.wifi_dashboard_button_position = self.wifi_dashboard_button_position[0] * self.row + self.wifi_dashboard_button_position[1]
        self.wifi_dashboard_button = self.create_button_img(self.wifi_dashboard_button_position, "WBD", os.path.join(self.asset_path, 'wifi_dashboard_default.png'))

        #### Video Dashboard Button
        self.video_dashboard_button_position = (0, 2)
        self.video_dashboard_button_position = self.video_dashboard_button_position[0] * self.row + self.video_dashboard_button_position[1]
        self.video_dashboard_button = self.create_button_img(self.video_dashboard_button_position, "VBD", os.path.join(self.asset_path, 'video_dashboard_default.png'))

        #### Camera Button
        self.camera_button_position = (0, 3)
        self.camera_button_position = self.camera_button_position[0] * self.row + self.camera_button_position[1]
        self.camera_button = self.create_button_img(self.camera_button_position, "Camera", os.path.join(self.asset_path, 'camera_default.png'))

        #### Arm Control Button
        self.arm_teleoperation_button_position = (1, 0)
        self.arm_teleoperation_button_position = self.arm_teleoperation_button_position[0] * self.row + self.arm_teleoperation_button_position[1]
        self.arm_teleoperation_button = self.create_button_img(self.arm_teleoperation_button_position, "WBD", os.path.join(self.asset_path, 'teleoperation_default.png'))

        #### Reset Odometry Button
        self.reset_odometry_button_position = (1,6)
        self.reset_odometry_button_position = self.reset_odometry_button_position[0] * self.row + self.reset_odometry_button_position[1]
        self.reset_odometry_button = self.create_button_img(self.reset_odometry_button_position, "Odometry", os.path.join(self.asset_path, 'reset_odometry_default.png'))

        #### Home Position Button
        self.home_position_button_position = (1, 7)
        self.home_position_button_position = self.home_position_button_position[0] * self.row + self.home_position_button_position[1]
        self.home_position_button = self.create_button_img(self.home_position_button_position, "Home", os.path.join(self.asset_path, 'home_default.png'))

        #### speech Button
        self.speech_button_position = (0, 6)
        self.speech_button_position = self.speech_button_position[0] * self.row + self.speech_button_position[1]
        self.speech_button = self.create_button_img(self.speech_button_position, "speech", os.path.join(self.asset_path, 'speech_default.png'))

        #### Speaker Button
        self.speaker_button_position = (0, 7)
        self.speaker_button_position = self.speaker_button_position[0] * self.row + self.speaker_button_position[1]
        self.speaker_button = self.create_button_img(self.speaker_button_position, "Speaker", os.path.join(self.asset_path, 'mic_default.png'))

        #### Record Button
        self.record_button_position = (3, 6)
        self.record_button_position = self.record_button_position[0] * self.row + self.record_button_position[1]
        self.record_button = self.create_button_img(self.record_button_position, "Record", os.path.join(self.asset_path, 'record_default.png'))

        #### Emergency Stop Button
        self.emergency_stop_button_position = (3, 7)
        self.emergency_stop_button_position = self.emergency_stop_button_position[0] * self.row + self.emergency_stop_button_position[1]
        self.emergency_stop_button = self.create_button(self.emergency_stop_button_position, "Emergency Stop", self.background_color_inactive)

        




if __name__ == '__main__':
    StreamDeckController()
