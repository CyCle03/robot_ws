import sys
import rclpy
from PySide6.QtWidgets import QApplication, QMainWindow
from .turtle_move_ui import Ui_MainWindow
from .move_turtlebot_pub import Move_turtle
from geometry_msgs.msg import Twist
from PySide6.QtCore import QThread, Signal, Slot
from rclpy.executors import MultiThreadedExecutor


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
    log_signal = Signal(str)

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
        self.log_signal.connect(self.append_log_line)


        #self.worker = QThread(targer=)
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.rclpy_thread = RclpyThread(self.executor)
        self.pub_move = Move_turtle()
        # cmd_vel을 기존 1에서 0.1(10hz)초 마다 명령을 보내도록 수정
        self.pub_move.timer = self.pub_move.create_timer(0.1, self.turtle_move)
        self.velocity = 0.0
        self.angular = 0.0
        self.angular_step = 0.2
        self.angular_limit = 1.0
        self.velocity_step = 0.2
        self.velocity_limit = 0.6
        self.enabled = False
        self.executor.add_node(self.pub_move)
        self.rclpy_thread.start()

    def turtle_move(self):
        if not self.enabled:
            return
        self.publish_cmd(self.velocity, self.angular)

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        self.pub_move.move_turtle.publish(msg)
        self.pub_move.get_logger().info(f'Published mesage: {msg.linear}, {msg.angular}')
        #자동스크롤
        self.log_signal.emit(f'{msg.linear.x}, {msg.angular.z}')

    @Slot(str)
    def append_log_line(self, text):
        self.ui.listWidget.addItem(text)
        self.ui.listWidget.scrollToBottom()

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def btn_stop_clicked(self):
        self.enabled = False
        self.velocity = 0.0
        self.angular = 0.0
        self.publish_cmd(self.velocity, self.angular)

    def btn_go_clicked(self):
        self.velocity = self.clamp(self.velocity + self.velocity_step, -self.velocity_limit , self.velocity_limit)
        self.enabled = True

    def btn_back_clicked(self):
        self.velocity = self.clamp(self.velocity - self.velocity_step, -self.velocity_limit , self.velocity_limit)
        self.enabled = True

    def btn_right_clicked(self):
        self.angular = self.clamp(self.angular - self.angular_step, -self.angular_limit, self.angular_limit)
        self.enabled = True

    def btn_left_clicked(self):
        self.angular = self.clamp(self.angular + self.angular_step, -self.angular_limit, self.angular_limit)
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
