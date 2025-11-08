import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopMover(Node):
    def __init__(self):
        super().__init__('teleop_mover')
        self.sub = self.create_subscription(Twist, '/cmd_vel_input', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Teleop mover ready — forwarding /cmd_vel_input → /cmd_vel")

    def callback(self, msg):
        self.pub.publish(msg)
        self.get_logger().info(
            f"Published cmd_vel: lin=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) "
            f"ang=({msg.angular.z:.2f})"
        )

def main():
    rclpy.init()
    node = TeleopMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
