import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState

class TeleopMover(Node):
    def __init__(self):
        super().__init__('teleop_mover')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.set_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.model_name = 'tello_drone'
        self.get_logger().info("Teleop mover ready.")

    def callback(self, msg):
        if not self.set_cli.wait_for_service(timeout_sec=1.0) or not self.get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Gazebo services not ready.")
            return

        get_req = GetEntityState.Request()
        get_req.name = self.model_name
        future_get = self.get_cli.call_async(get_req)
        rclpy.spin_until_future_complete(self, future_get)
        current_state = future_get.result().state

        current_state.pose.position.x += msg.linear.x * 0.1
        current_state.pose.position.y += msg.linear.y * 0.1
        current_state.pose.position.z += msg.linear.z * 0.1

        set_req = SetEntityState.Request()
        set_req.state = current_state
        set_req.state.name = self.model_name

        future_set = self.set_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, future_set)

def main():
    rclpy.init()
    node = TeleopMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
