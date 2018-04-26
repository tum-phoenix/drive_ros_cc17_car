import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
#from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtWidgets import QWidget

from .heartbeat_widget import HeartbeatWidget

class HeartbeatVisualization(Plugin):


    def __init__(self, context):
        super(HeartbeatVisualization, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('HeartbeatVisualization')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = HeartbeatWidget()
        # Get path to UI file which should be in the "resource" folder of this package
   #     ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'heartbeat.ui')
        # Extend the widget with all attributes and children from UI file
   #     loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
   #     self._widget.setObjectName('HeartbeatVisualizationUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._widget.shutdown_plugin()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

