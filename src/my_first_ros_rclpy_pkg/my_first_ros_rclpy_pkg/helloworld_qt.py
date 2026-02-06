import sys
import rclpy
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QFile
from .msg_qt_ui import Ui_MainWindow
from .helloworld_publisher import HelloworldPublisher
from .helloworld_subscriber import HelloworldSubscriber
from PySide6.QtCore import QThread, Signal, Slot
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rclpy.node import Node


class RclpyThread(QThread):
    def __init__(self, executor):
        super().__init__()
        self.executor = executor

    def run(self):
        try:
            self.executor.spin()
        finally:
            rclpy.shutdown()


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        # setupUi 함수를 호출해 MainWindow에 있는 위젯을 배치한다.
        self.ui.setupUi(self)
        # button clicked 이벤트 핸들러로 button_clicked 함수와 연결한다.
        self.ui.btn_publisher.clicked.connect(self.btn_publisher_clicked)
        self.ui.btn_subscriber.clicked.connect(self.btn_subscriber_clicked)
        self.ui.btn_pub_cancel.clicked.connect(self.btn_pub_cancel_clicked)
        self.ui.btn_sub_cancel.clicked.connect(self.btn_sub_cancel_clicked)
        #self.worker = QThread(targer=)
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.rclpy_thread = RclpyThread(self.executor)
        self.pub = HelloworldPublisher()
        self.sub = HelloworldSubscriber()
        self.sub.helloworld_subscriber.callback = self.subscribe_topic_message
        self.rclpy_thread.start()


    def btn_publisher_clicked(self):
        self.executor.add_node(self.pub)

    def btn_subscriber_clicked(self):
        self.executor.add_node(self.sub)

    def btn_pub_cancel_clicked(self):
        self.executor.remove_node(self.pub)

    def btn_sub_cancel_clicked(self):
        self.executor.remove_node(self.sub)

#서브스크라이버 노드 콜백함수 재정의
    def subscribe_topic_message(self, msg):
        rcv_msg = 'Received message: {0}'.format(msg.data)
        self.sub.get_logger().info(rcv_msg)
        self.ui.msg_list.addItem(rcv_msg)

    def closeEvent(self, event):
        # 종료 시 리소스 정리
        self.executor.shutdown()
        self.rclpy_thread.quit()
        self.rclpy_thread.wait()
        super().closeEvent(event)

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
