import math
import time

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

def thumb_is_open(landmarks):
    palm_x = (landmarks[0].x + landmarks[9].x) / 2
    palm_y = (landmarks[0].y + landmarks[9].y) / 2

    thumb_tip = landmarks[4]
    thumb_mcp = landmarks[2]

    dist_tip = math.hypot(thumb_tip.x - palm_x, thumb_tip.y - palm_y)
    dist_mcp = math.hypot(thumb_mcp.x - palm_x, thumb_mcp.y - palm_y)
    # 엄지 오검출을 줄이기 위해 기준을 조금 더 보수적으로 둔다.
    return dist_tip > dist_mcp * 1.35


def classify_hand(hand_landmarks):
    landmarks = hand_landmarks.landmark

    def is_finger_up(finger_tip_idx, finger_dip_idx, margin=0.015):
        return landmarks[finger_tip_idx].y < (landmarks[finger_dip_idx].y - margin)

    def is_finger_down(finger_tip_idx, finger_dip_idx, margin=0.01):
        return landmarks[finger_tip_idx].y > (landmarks[finger_dip_idx].y + margin)

    index_straight = is_finger_up(8, 6)
    middle_straight = is_finger_up(12, 10)
    ring_straight = is_finger_up(16, 14)
    pinky_straight = is_finger_up(20, 18)
    middle_folded = is_finger_down(12, 10)
    ring_folded = is_finger_down(16, 14)
    pinky_folded = is_finger_down(20, 18)
    thumb_open = thumb_is_open(landmarks)

    all_folded = (
        not index_straight and middle_folded and ring_folded and pinky_folded
    )

    if all_folded:
        return "rock"
    if index_straight and middle_folded and ring_folded and pinky_folded:
        return "one"
    if index_straight and middle_straight and ring_folded and pinky_folded:
        return "scissors"
    if index_straight and middle_straight and ring_straight and not pinky_straight:
        return "three"
    if index_straight and middle_straight and ring_straight and pinky_straight:
        return "paper" if thumb_open else "four"
    return "unknown"

class HandGestureTurtleController(Node):
    def __init__(self):
        super().__init__('hand_gesture_turtle')
        self.qos_profile = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)

        self.velocity = 0.0
        self.angular = 0.0
        self.forward_velocity = 0.1
        self.backward_velocity = -0.1
        self.left_angular = 0.6
        self.right_angular = -0.6
        self.enabled = False
        self.publish_period_sec = 0.1

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(value, high))

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z

        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f"Published message: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}"
        )

    def turtle_stop(self):
        self.enabled = False
        self.velocity = 0.0
        self.angular = 0.0
        self.publish_cmd(self.velocity, self.angular)

    def turtle_go(self):
        self.velocity = self.forward_velocity
        self.angular = 0.0
        self.enabled = True

    def turtle_back(self):
        self.velocity = self.backward_velocity
        self.angular = 0.0
        self.enabled = True

    def turtle_right(self):
        self.velocity = 0.0
        self.angular = self.right_angular
        self.enabled = True

    def turtle_left(self):
        self.velocity = 0.0
        self.angular = self.left_angular
        self.enabled = True

    def apply_gesture(self, gesture):
        if gesture == "rock":
            self.turtle_stop()
        elif gesture == "one":
            self.turtle_go()
        elif gesture == "scissors":
            self.turtle_left()
        elif gesture in {"three", "four"}:
            self.turtle_right()
        elif gesture == "paper":
            self.turtle_back()

    def run(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("카메라를 열 수 없습니다.")

        last_applied_gesture = None
        last_publish_time = 0.0

        try:
            with mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            ) as hands:
                while cap.isOpened():
                    success, image = cap.read()
                    if not success:
                        continue

                    image = cv2.flip(image, 1)
                    image.flags.writeable = False
                    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    results = hands.process(rgb_image)

                    image.flags.writeable = True

                    gesture = "no_hand"
                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            mp_drawing.draw_landmarks(
                                image,
                                hand_landmarks,
                                mp_hands.HAND_CONNECTIONS,
                                mp_drawing_styles.get_default_hand_landmarks_style(),
                                mp_drawing_styles.get_default_hand_connections_style(),
                            )
                            gesture = classify_hand(hand_landmarks)

                    if gesture == "no_hand":
                        last_applied_gesture = None
                    elif gesture != last_applied_gesture and gesture != "unknown":
                        self.apply_gesture(gesture)
                        last_applied_gesture = gesture

                    now = time.monotonic()
                    if now - last_publish_time >= self.publish_period_sec:
                        if self.enabled:
                            self.publish_cmd(self.velocity, self.angular)
                        else:
                            self.publish_cmd(0.0, 0.0)
                        last_publish_time = now

                    cv2.putText(
                        image,
                        gesture,
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.2,
                        (0, 255, 0),
                        2,
                    )
                    cv2.imshow("MediaPipe Hands", image)

                    if cv2.waitKey(5) & 0xFF == 27:
                        break
        finally:
            self.turtle_stop()
            cap.release()
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    controller = HandGestureTurtleController()
    try:
        controller.run()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
