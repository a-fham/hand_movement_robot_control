import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HandTrackingNode(Node):

    def __init__(self):
        super().__init__('task2_node')
        self.publisher_ = self.create_publisher(String, 'hand_position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to grab frame')
            return
        frame = cv2.flip(frame, 1)
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)
        position = 'Unknown'
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                position = self.get_hand_position(hand_landmarks)
                cv2.putText(frame, position, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                self.publish_hand_position(position)
                print(f"Hand position detected: {position}")  # Debugging print
        cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 255, 0), 2)
        cv2.line(frame, (0, frame.shape[0] // 2), (frame.shape[1], frame.shape[0] // 2), (255, 0, 0), 2)
        cv2.imshow('Hand Tracking', frame)
        print("Frame displayed")  # Debugging print
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def get_hand_position(self, hand_landmarks):
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        if index_tip.y > 0.5 and index_tip.x > 0.5:
            return "Backward"
        elif index_tip.y > 0.5 and index_tip.x < 0.5:
            return "Forward"
        elif index_tip.x < 0.5 and index_tip.y < 0.5:
            return "Left"
        elif index_tip.x > 0.5 and index_tip.y < 0.5:
            return "Right"
        else:
            return "Unknown"

    def publish_hand_position(self, position):
        msg = String()
        msg.data = position
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
