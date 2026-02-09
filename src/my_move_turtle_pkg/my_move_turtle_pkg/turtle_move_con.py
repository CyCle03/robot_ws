import sys
import rclpy
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QFile
from .turtle_move_ui import Ui_MainWindow
from .move_turtle_pub import Move_turtle
from geometry_msgs.msg import Twist
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
        self.ui.btn_go.clicked.connect(self.btn_go_clicked)
        self.ui.btn_back.clicked.connect(self.btn_back_clicked)
        self.ui.btn_right.clicked.connect(self.btn_right_clicked)
        self.ui.btn_left.clicked.connect(self.btn_left_clicked)
        self.ui.btn_stop.clicked.connect(self.btn_stop_clicked)


        #self.worker = QThread(targer=)
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.rclpy_thread = RclpyThread(self.executor)
        self.pub_move = Move_turtle()
        self.pub_move.timer = self.pub_move.create_timer(1, self.turtle_move)
        self.velocity = 0.0
        self.angular = 0.0
        self.enabled = False
        self.rclpy_thread.start()
        self.executor.add_node(self.pub_move)


    def turtle_move(self):
        if not self.enabled:
            return
        msg = Twist()
        msg.linear.x = self.velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular
        self.pub_move.move_turtle.publish(msg)
        self.pub_move.get_logger().info(f'Published mesage: {msg.linear}, {msg.angular}')
        self.ui.listWidget.addItem(f'{msg.linear._x}, {msg.angular._z}')

    def btn_stop_clicked(self):
        self.velocity = 0.0
        self.angular = 0.0
        self.enabled = False

    def btn_go_clicked(self):
        self.velocity += 0.2
        self.enabled = True

    def btn_back_clicked(self):
        self.velocity -= 0.2
        self.enabled = True

    def btn_right_clicked(self):
        self.angular -= 0.2
        self.enabled = True

    def btn_left_clicked(self):
        self.angular += 0.2
        self.enabled = True


    def ros_executer(self):
        self.executor.spin()

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
