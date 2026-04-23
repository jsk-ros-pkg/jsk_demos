import rospy
from python_qt_binding.QtWidgets import QWidget, QGridLayout, QPushButton, QSizePolicy
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String, UInt16


class ActionButton(Plugin):
    def __init__(self, context):
        super(ActionButton, self).__init__(context)
        self.setObjectName("ActionButton")

        # Create QWidget for the GUI
        self._widget = QWidget()
        self._widget.setWindowTitle("Action Button GUI")

        # Grid Layout
        self.layout = QGridLayout(self._widget)
        self._widget.setLayout(self.layout)

        self.publisher = rospy.Publisher("/action_trigger", UInt16, queue_size=1)

        # Mapping for button labels
        button_labels = {
            6: "Thanks",
            7: "Bye (R)",
            8: "Bye (L)",
            9: "Hi (R)",
            10: "Hi (L)",
            11: "Wait",
            12: "Stomach",
            13: "Oops",
            14: "Sleepy",
            15: "OK",
            16: "Yes",
            17: "Right",
            18: "Understand",
            19: "Disagree",
            20: "Don't know",
            21: "Honey",
            22: "Janken",
            23: "Aiko",
            24: "Lose",
            25: "Yeah",
            26: "End Janken",
            27: "Banzai",
            28: "Start Shake",
            29: "During Shake",
            30: "End Shake"
        }

        # Create buttons
        for row in range(6): # 4
            for col in range(5):
                button_number = row * 5 + col + 1
                button = QPushButton(
                    button_labels.get(button_number, str(button_number))
                )  # Use label if available, otherwise default to number
                button.setSizePolicy(
                    QSizePolicy.Expanding, QSizePolicy.Expanding
                )  # Allow buttons to expand
                button.setMinimumSize(50, 50)
                button.clicked.connect(self.create_button_callback(button_number))
                self.layout.addWidget(button, row, col)

        # Add the widget to the context
        context.add_widget(self._widget)

    def create_button_callback(self, button_number):
        def callback():
            rospy.loginfo(f"Button {button_number} pressed")
            msg = UInt16(button_number)
            self.publisher.publish(msg)

        return callback
