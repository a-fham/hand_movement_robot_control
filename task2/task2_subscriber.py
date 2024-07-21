import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import time

class RobotControlNode(Node):

    def __init__(self):
        super().__init__('task2_subscriber')
        self.subscription = self.create_subscription(String, 'hand_position', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize the last command time
        self.last_command_time = time()
        
        # Create a timer that calls the `timer_callback` method periodically
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust the timer period if needed

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
        twist = Twist()
        if msg.data == 'Forward':
            twist.linear.x = 0.3
        elif msg.data == 'Backward':
            twist.linear.x = -0.3
        elif msg.data == 'Left':
            twist.angular.z = 0.3
        elif msg.data == 'Right':
            twist.angular.z = -0.3
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Update the last command time
        self.last_command_time = time()
        self.publisher_.publish(twist)

    def timer_callback(self):
        current_time = time()
        # If the time since the last command exceeds the timeout period, stop the robot
        if current_time - self.last_command_time > 0.5:  # Timeout period of 1 second
            twist = Twist()
            self.publisher_.publish(twist)  # Stop the robot

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
