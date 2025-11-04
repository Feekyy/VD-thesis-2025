import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class TeleopMover(Node):
    def __init__(self):
        super().__init__('teleop_mover')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_logger().info("Teleop mover ready.")

    def callback(self, msg):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not ready")
            return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = "cam"
        req.state.pose.position.x += msg.linear.x * 0.1
        req.state.pose.position.y += msg.linear.y * 0.1
        req.state.pose.position.z += msg.linear.z * 0.1

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main():
    rclpy.init()
    node = TeleopMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
