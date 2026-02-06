import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtCore import QFile, QThread, Signal, Slot
from rclpy.executors import MultiThreadedExecutor
from .sub_msg_ui import Ui_Form

class RclpyThread(QThread):
    def __init__(self, executor):
        super().__init__()
        self.executor = executor

    def run(self):
        try:
            self.executor.spin()
        finally:
            rclpy.shutdown()

class HelloworldSubscriber(QWidget):
    def __init__(self):
        super(HelloworldSubscriber, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui.btn_start.clicked.connect(self.btn_sub_start_clicked)
        self.ui.btn_cancel.clicked.connect(self.btn_sub_cancel_clicked)
        rclpy.init()
        self.sub_node = Node("Helloworld_subscriber")
        qos_profile = QoSProfile(depth=10)
        self.helloworld_subscriber = self.sub_node.create_subscription(
            String,
            'helloworld',
            self.subscribe_topic_message,
            qos_profile)
        self.executor = MultiThreadedExecutor()
        self.rclpy_thread = RclpyThread(self.executor)
        self.rclpy_thread.start()

    def subscribe_topic_message(self, msg):
        self.sub_node.get_logger().info('Received message: {0}'.format(msg.data))
        self.ui.listWidget.addItem(msg.data)

    def btn_sub_start_clicked(self):
        self.executor.add_node(self.sub_node)

    def btn_sub_cancel_clicked(self):
        self.executor.remove_node(self.sub_node)

    def closeEvent(self, event):
        # 종료 시 리소스 정리
        self.executor.shutdown()
        self.rclpy_thread.quit()
        self.rclpy_thread.wait()
        super().closeEvent(event)


    def closeEvent(self, event):
        # 종료 시 리소스 정리
        print("쓰레드 및 노드 종료")
        self.executor.shutdown()
        self.rclpy_thread.quit()
        self.rclpy_thread.wait()
        self.pub_node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

def main():
    app = QApplication(sys.argv)
    window = HelloworldSubscriber()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
