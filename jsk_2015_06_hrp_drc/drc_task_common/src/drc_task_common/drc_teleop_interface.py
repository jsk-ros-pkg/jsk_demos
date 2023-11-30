import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import python_qt_binding.QtCore as QtCore
from python_qt_binding.QtGui import *
from drc_task_common.srv import EusCommand

class DRCTeleopInterface(Plugin):

    def __init__(self, context):
        super(DRCTeleopInterface, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('drc_teleop_interface')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'config', 'drc_teleop_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('drc_teleop_interfaceUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ROS service client
        global eus_commmand_srv
        eus_commmand_srv = rospy.ServiceProxy('/eus_command', EusCommand)

        # Robot Pose Button
        ## Reset Pose
        self._widget.reset_pose_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'reset-pose.jpg'))))
        self._widget.reset_pose_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.reset_pose_button.clicked[bool].connect(self.reset_pose_cb)
        ## Reset Manip Pose
        self._widget.reset_manip_pose_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'reset-manip-pose.jpg'))))
        self._widget.reset_manip_pose_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.reset_manip_pose_button.clicked[bool].connect(self.reset_manip_pose_cb)
        ## Init Pose
        self._widget.init_pose_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'init-pose.jpg'))))
        self._widget.init_pose_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.init_pose_button.clicked[bool].connect(self.init_pose_cb)

        # Hand Pose Button
        ## Reset Pose
        self._widget.hand_reset_pose_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'hand-reset-pose.jpg'))))
        self._widget.hand_reset_pose_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hand_reset_pose_button.clicked[bool].connect(self.hand_reset_pose_cb)
        ## Hook Pose
        self._widget.hand_hook_pose_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'hand-hook-pose.jpg'))))
        self._widget.hand_hook_pose_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hand_hook_pose_button.clicked[bool].connect(self.hand_hook_pose_cb)
        ## Grasp Pose
        self._widget.hand_grasp_pose_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'hand-grasp-pose.jpg'))))
        self._widget.hand_grasp_pose_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hand_grasp_pose_button.clicked[bool].connect(self.hand_grasp_pose_cb)

        # Hrpsys Button
        ## Start ABC
        self._widget.hrpsys_start_abc_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'start-abc.png'))))
        self._widget.hrpsys_start_abc_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hrpsys_start_abc_button.clicked[bool].connect(self.hrpsys_start_abc_cb)
        ## Start ST
        self._widget.hrpsys_start_st_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'start-st.png'))))
        self._widget.hrpsys_start_st_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hrpsys_start_st_button.clicked[bool].connect(self.hrpsys_start_st_cb)
        ## Start IMP
        self._widget.hrpsys_start_imp_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'start-imp.png'))))
        self._widget.hrpsys_start_imp_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hrpsys_start_imp_button.clicked[bool].connect(self.hrpsys_start_imp_cb)
        ## Stop ABC
        self._widget.hrpsys_stop_abc_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'stop-abc.png'))))
        self._widget.hrpsys_stop_abc_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hrpsys_stop_abc_button.clicked[bool].connect(self.hrpsys_stop_abc_cb)
        ## Stop ST
        self._widget.hrpsys_stop_st_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'stop-st.png'))))
        self._widget.hrpsys_stop_st_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hrpsys_stop_st_button.clicked[bool].connect(self.hrpsys_stop_st_cb)
        ## Stop IMP
        self._widget.hrpsys_stop_imp_button.setIcon(QIcon(QPixmap(os.path.join(rospkg.RosPack().get_path('drc_task_common'), 'icons', 'stop-imp.png'))))
        self._widget.hrpsys_stop_imp_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self._widget.hrpsys_stop_imp_button.clicked[bool].connect(self.hrpsys_stop_imp_cb)

    # Robot Pose Button Callback
    def reset_pose_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :angle-vector (send *robot* :reset-pose) 5000)')
        print('send reset_pose command')
    def reset_manip_pose_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)')
        print('send reset_manip_pose command')
    def init_pose_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :angle-vector (send *robot* :init-pose) 5000)')
        print('send init_pose command')

    # Hand Pose Button Callback
    def hand_reset_pose_cb(self, checked):
        eus_commmand_srv(command='(progn (send *robot* :hand :arms :reset-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))))')
        print('send hand_reset_pose command')
    def hand_hook_pose_cb(self, checked):
        eus_commmand_srv(command='(progn (send *robot* :hand :arms :hook-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))))')
        print('send hand_hook_pose command')
    def hand_grasp_pose_cb(self, checked):
        eus_commmand_srv(command='(progn (send *robot* :hand :arms :grasp-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))))')
        print('send hand_grasp_pose command')

    # Hrpsys Button Callback
    def hrpsys_start_abc_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :start-auto-balancer)')
        print('send hrpsys_start_abc command')
    def hrpsys_start_st_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :start-st)')
        print('send hrpsys_start_st command')
    def hrpsys_start_imp_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :start-impedance :arms)')
        print('send hrpsys_start_imp command')
    def hrpsys_stop_abc_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :stop-auto-balancer)')
        print('send hrpsys_stop_abc command')
    def hrpsys_stop_st_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :stop-st)')
        print('send hrpsys_stop_st command')
    def hrpsys_stop_imp_cb(self, checked):
        eus_commmand_srv(command='(send *ri* :stop-impedance :arms)')
        print('send hrpsys_stop_imp command')

    def shutdown_plugin(self):
        eus_commmand_srv.close()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
