#!/usr/bin/env python

from os.path import join

import rospkg
from PyQt5 import QtGui
import PyQt5.uic

import rospy
from PyQt5.QtWidgets import QLabel, QWidget

from drive_ros_msgs.msg import mav_cc16_HEARTBEAT

class HeartbeatWidget(QWidget):

    #TODO get from somewhere else?
    remote_control_status = ('DISCONNECTED', 'CONNECTED','AUTONOMOUS', 'SEMI_AUTONOMOUS', 'MANUAL')
    drive_mode_status = ('TRACK', 'TRACK_OBSTACLES', 'PARKING', 'IDLE')

    def callback(self, heartbeat):
        # battery
        if self.old_battery_voltage != heartbeat.battery_voltage:
            self.batterybar.setValue(heartbeat.battery_voltage)
            self.battery_voltage_displayValue.setText(str(heartbeat.battery_voltage) + " mV")
            self.old_battery_voltage = heartbeat.battery_voltage

        # velocity
        if self.old_speed != heartbeat.rc_velocity:
            speed = int(abs(heartbeat.rc_velocity) * 100)
            self.speedbar.setValue(speed)
            self.rc_velocity_displayValue.setText("%.3f m/s" % heartbeat.rc_velocity)
            self.old_speed = heartbeat.rc_velocity

        # drive mode
        if self.old_drive_mode != heartbeat.drive_mode:
            self.drive_mode_displayValue.setText(str(heartbeat.drive_mode) + " - " + self.drive_mode_status[heartbeat.drive_mode])
            self.old_drive_mode = heartbeat.drive_mode

        # remote control status
        if self.old_remote_control != heartbeat.remote_control:
            self.remote_control_displayValue.setText(str(heartbeat.remote_control) + " - " + self.remote_control_status[heartbeat.remote_control])
            self.old_remote_control = heartbeat.remote_control

        # tires
        # front wheels
        if self.old_front_angle != heartbeat.rc_steering_front:
            self.rc_steering_front_displayValue.setText("%.3f rad" % heartbeat.rc_steering_front)
            tr = QtGui.QTransform()
            tr.rotateRadians(heartbeat.rc_steering_front)
            rotated = self.tire.transformed(tr)
            self.right_front_wheel.setPixmap(rotated)
            self.left_front_wheel.setPixmap(rotated)
            self.old_front_angle = heartbeat.rc_steering_front
        # rear wheels
        if self.old_rear_angle != heartbeat.rc_steering_rear:
            self.rc_steering_rear_displayValue.setText("%.3f rad" % heartbeat.rc_steering_rear)
            tr = QtGui.QTransform()
            tr.rotateRadians(heartbeat.rc_steering_rear)
            rotated = self.tire.transformed(tr)
            self.right_rear_wheel.setPixmap(rotated)
            self.left_rear_wheel.setPixmap(rotated)
            self.old_rear_angle = heartbeat.rc_steering_rear

        # direction
        if heartbeat.rc_velocity > 0 and self.old_direction != 1:
            self.driving_direction_backwards.hide()
            self.driving_direction_forward.show()
            self.old_direction = 1
        elif heartbeat.rc_velocity < 0 and self.old_direction != -1:
            self.driving_direction_forward.hide()
            self.driving_direction_backwards.show()
            self.old_direction = -1
        elif heartbeat.rc_velocity == 0 and self.old_direction != 0:
            self.driving_direction_backwards.hide()
            self.driving_direction_forward.hide()
            self.old_direction = 0

    def __init__(self):

        super(HeartbeatWidget, self).__init__()

        # init old values
        # sometimes invalid value, so it's set on first message
        self.old_front_angle = float(-1)
        self.old_rear_angle = float(-1)
        self.old_battery_voltage = 0
        self.old_speed = float(-1)
        self.old_drive_mode = -1
        self.old_remote_control = -1
        self.old_direction = 0  # 0 = stop, -1 = backwards, 1 = forwards


        # ui
        ui_file = join(rospkg.RosPack().get_path('drive_ros_cc2017_car'), 'resource', 'heartbeat.ui')
        PyQt5.uic.loadUi(ui_file, self)

        # Give QObjects reasonable names
        self.setObjectName('HeartbeatVisualizationUi')

        # init widgets
        self.tire_file = join(rospkg.RosPack().get_path('drive_ros_cc2017_car'), 'resource', 'tire_small.png')
        self.tire = QtGui.QPixmap(self.tire_file)
        self.right_front_wheel.setPixmap(self.tire)
        self.left_front_wheel.setPixmap(self.tire)
        self.right_rear_wheel.setPixmap(self.tire)
        self.left_rear_wheel.setPixmap(self.tire)

        self.speedbar = self.rc_velocity_progressBar
        self.speedbar.setRange(0,rospy.get_param("~max_velocity"))

        self.batterybar = self.battery_voltage_progressBar
        self.batterybar.setRange(0,rospy.get_param("~max_voltage"))

        # direction
        self.arrow_file = join(rospkg.RosPack().get_path('drive_ros_cc2017_car'), 'resource', 'arrow.png')
        direction = QtGui.QPixmap(self.arrow_file)
        self.driving_direction_forward.setPixmap(direction)
        self.driving_direction_forward.hide()
        tr = QtGui.QTransform()
        tr.rotate(float(180))
        self.driving_direction_backwards.setPixmap(direction.transformed(tr))
        self.driving_direction_backwards.hide()

        # subscribe to heartbeat
        self._heartbeat_sub = rospy.Subscriber("/from_mav/heartbeat", mav_cc16_HEARTBEAT, self.callback)

    def shutdown_plugin(self):
        self._heartbeat_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

