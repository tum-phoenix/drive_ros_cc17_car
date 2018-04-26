#!/usr/bin/env python

import os

import rospkg
from PyQt5 import QtGui
import PyQt5.uic

import rospy
from PyQt5.QtWidgets import QLabel, QWidget

#from qt_gui.plugin import Plugin
#from python_qt_binding import loadUi
#from python_qt_binding.QtGui import QWidget
#from python_qt_binding.QtWidgets import QWidget

from drive_ros_msgs.msg import mav_cc16_HEARTBEAT

class HeartbeatWidget(QWidget):

    #TODO entfernen?
    _heartbeat_attributes = ('battery_voltage', 'drive_mode', 'rc_steering_front', 'rc_steering_rear', 'remote_control')
    remote_control_status = ('DISCONNECTED', 'CONNECTED','AUTONOMOUS', 'SEMI_AUTONOMOUS', 'MANUAL')
    drive_mode_status = ('TRACK', 'TRACK_OBSTACLES',
                         'PARKING', 'IDLE')

    def callback(self, heartbeat):
        if self.old_battery_voltage != heartbeat.battery_voltage:
            self.batterybar.setValue(heartbeat.battery_voltage) #TODO sometimes not set?
            self.battery_voltage_displayValue.setText(str(heartbeat.battery_voltage))
            self.old_battery_voltage = heartbeat.battery_voltage

        if self.old_speed != heartbeat.rc_velocity:
            speed = int(abs(heartbeat.rc_velocity) * 100)
            # TODO speedbar.setValue() crashes whole thing
            #self.speedbar.setValue(speed) #TODO m/s
            self.rc_velocity_displayValue.setText(str(heartbeat.rc_velocity))
            self.old_speed = heartbeat.rc_velocity

        if self.old_drive_mode != heartbeat.drive_mode:
            self.drive_mode_displayValue.setText(str(heartbeat.drive_mode) + " - " + self.drive_mode_status[heartbeat.drive_mode])
            self.old_drive_mode = heartbeat.drive_mode

        if self.old_remote_control != heartbeat.remote_control:
            self.remote_control_displayValue.setText(str(heartbeat.remote_control) + " - " + self.remote_control_status[heartbeat.remote_control])
            self.old_remote_control = heartbeat.remote_control

        #tires TODO dont draw new every time
        # front wheels
        if self.old_front_angle != heartbeat.rc_steering_front:
            self.rc_steering_front_displayValue.setText(str(heartbeat.rc_steering_front))
            tr = QtGui.QTransform()
            tr.rotateRadians(heartbeat.rc_steering_front)
            rotated = self.tire.transformed(tr)
            self.right_front_wheel.setPixmap(rotated)
            self.left_front_wheel.setPixmap(rotated)
            self.old_front_angle = heartbeat.rc_steering_front
        # rear wheels
        if self.old_rear_angle != heartbeat.rc_steering_rear:
            self.rc_steering_rear_displayValue.setText(str(heartbeat.rc_steering_rear))
            tr = QtGui.QTransform()
            tr.rotateRadians(heartbeat.rc_steering_rear)
            rotated = self.tire.transformed(tr)
            self.right_rear_wheel.setPixmap(rotated)
            self.left_rear_wheel.setPixmap(rotated)
            self.old_rear_angle = heartbeat.rc_steering_rear

        #show direction TODO doesn't hide correctly
        if heartbeat.rc_velocity > 0:
            self.driving_direction_backwards.hide()
            self.driving_direction_forward.show()
        elif heartbeat.rc_velocity < 0:
            self.driving_direction_forward.hide()
            self.driving_direction_backwards.show()
        else:
            self.driving_direction_backwards.hide()
            self.driving_direction_forward.hide()

    def __init__(self):

        super(HeartbeatWidget, self).__init__()

        # old values
        self.old_front_angle = float(0)
        self.old_rear_angle = float(0)
        self.old_battery_voltage = 0
        self.old_speed = float(0)
        self.old_drive_mode = 0
        self.old_remote_control = 0
        self.old_direction = 0  # 0 = stop, -1 = backwards, 1 = forwards


        # TODO PATH
        ui_file = os.path.join(rospkg.RosPack().get_path('drive_ros_cc2017_car'), 'resource', 'heartbeat.ui')
        PyQt5.uic.loadUi(ui_file, self)

        # Give QObjects reasonable names
        self.setObjectName('HeartbeatVisualizationUi')

        # TODO PATH
        self.tire_file = os.path.join(rospkg.RosPack().get_path('drive_ros_cc2017_car'), 'resource', 'tire_small.png')
        self.tire = QtGui.QPixmap(self.tire_file)
        self.right_front_wheel.setPixmap(self.tire)
        self.left_front_wheel.setPixmap(self.tire)
        self.right_rear_wheel.setPixmap(self.tire)
        self.left_rear_wheel.setPixmap(self.tire)
        # TODO add speedometer widget in heartbeat.ui with sip?

        self.speedbar = self.rc_velocity_progressBar
        self.speedbar.setRange(0,200) #TODO get from launch file

        self.batterybar = self.battery_voltage_progressBar
        self.batterybar.setRange(0,10000) #TODO get from launch file

        # direction
        self.arrow_file = os.path.join(rospkg.RosPack().get_path('drive_ros_cc2017_car'), 'resource', 'arrow.png')
        direction = QtGui.QPixmap(self.arrow_file)  # TODO set in launch file
        self.driving_direction_forward.setPixmap(direction)
        self.driving_direction_forward.hide()
        tr = QtGui.QTransform()
        tr.rotate(float(180))
        self.driving_direction_backwards.setPixmap(direction.transformed(tr))
        self.driving_direction_backwards.hide()

        global _heartbeat_sub
        _heartbeat_sub = rospy.Subscriber("/from_mav/heartbeat", mav_cc16_HEARTBEAT, self.callback)

    def shutdown_plugin(self):
        global _heartbeat_sub
        _heartbeat_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        pass #TODO?

    def restore_settings(self, plugin_settings, instance_settings):
        pass #TODO?

