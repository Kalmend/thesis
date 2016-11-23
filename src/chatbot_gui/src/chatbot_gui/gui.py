import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from python_qt_binding.QtGui import QStandardItemModel, QStandardItem

try:
    from python_qt_binding.QtGui import QWidget, QPlainTextEdit
except ImportError:
    from python_qt_binding.QtWidgets import QWidget, QPlainTextEdit


from python_qt_binding.QtCore import Qt, QVariant, QDateTime
from .nav_view import NavViewWidget
from std_msgs.msg import String

class ChatbotGUI(Plugin):

    def __init__(self, context):
        super(ChatbotGUI, self).__init__(context)
        self.setObjectName('ChatbotGUI')
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('chatbot_gui'), 'resource', 'ChatbotGUI.ui')
        loadUi(ui_file, self._widget, {'NavViewWidget': NavViewWidget})
        self._widget.setObjectName('ChatbotGUIUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
	
        self._widget.pickDone.clicked[bool].connect(self._handle_pick_clicked)
        self._widget.placeDone.clicked[bool].connect(self._handle_place_clicked)
	self._widget.textSay.keyPressEvent = self._handle_custom_keypress
	#ros stuff
	self.pubPick = rospy.Publisher('picker', String, queue_size=10)
	self.pubPlace = rospy.Publisher('placer', String, queue_size=10)
	self.pubRecog = rospy.Publisher('recognition/raw_result', String, queue_size=10)
	self.subRecog = rospy.Subscriber("recognition/raw_result", String, self._recog_callback)
	self.subLog = rospy.Subscriber("chatbot_gui/log", String, self._log_callback)
	self.subStatus = rospy.Subscriber("chatbot_gui/status", String, self._status_callback)

	self.model = QStandardItemModel()
	self.model.setColumnCount(2)
	headerNames = []
	headerNames.append("Timestamp")
	headerNames.append("Message")
	self.model.setHorizontalHeaderLabels(headerNames)
	self._widget.tableLog.setModel(self.model)
	self._widget.tableLog.horizontalHeader().setStretchLastSection(True)
	self._widget.tableLog.setColumnWidth(0, 170)
	self._message_limit = 20000
	self._message_count = 0


    def _add_log(self, data):
	timestamp = QStandardItem(QDateTime.currentDateTime().toString('d.M.yyyy hh:mm:ss.zzz'))
	timestamp.setTextAlignment(Qt.AlignRight)
	timestamp.setEditable(False)

        message = QStandardItem(data)
        message.setEditable(False)

	row = []
        row.append(timestamp)

	row.append(message)

	self._message_count = self._message_count + 1
	self.model.insertRow(0, row)
	
	if self._message_count > self._message_limit:
		self.model.removeRow(self.model.rowCount()-1)
		self._message_count = self._message_count - 1

    def _handle_pick_clicked(self):
        self.pubPick.publish("[pick done]") 

    def _handle_place_clicked(self):
        self.pubPlace.publish("[place done]") 
 
    def _handle_custom_keypress(self, event):
	if event.key() == Qt.Key_Return or event.key() == Qt.Key_Enter: 
        	
		self.pubRecog.publish(self._widget.textSay.toPlainText())
		self._widget.textSay.setPlainText("")
	else:
		QPlainTextEdit.keyPressEvent(self._widget.textSay, event)

    def _recog_callback(self, data):
    	self._add_log("heard: '" + data.data + "'")

    def _log_callback(self, data):
    	self._add_log(data.data)

    def _status_callback(self, data):
    	self._widget.labelStatus.setText(data.data)

    def shutdown_plugin(self):
	print("shutdown_plugin: clearing ros publishers.")
	self.subRecog.unregister()
	self.subStatus.unregister()
	self.subLog.unregister()
        self.pubPick.unregister()
	self.pubPlace.unregister()
	self.pubRecog.unregister()
        pass

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
