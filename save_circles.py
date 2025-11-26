import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CircleSaver(Node):
    def __init__(self):
        super().__init__('circle_saver')
        self.sub_circles = self.create_subscription(
            String,
            '/donut_detector/circles',
            self.callback_circles,
            10)
            
        self.sub_info = self.create_subscription(
            String,
            '/donut_detector/info',
            self.callback_info,
            10)
            
        self.file = open("output.txt", "w")
        self.get_logger().info("Saving data to output.txt... (Press Ctrl+C to stop)")

    def callback_circles(self, msg):
        line = f"[CIRCLES]: {msg.data}"
        self.write_to_file(line)
        print(f"Saved CIRCLES message length: {len(msg.data)}")

    def callback_info(self, msg):
        line = f"[INFO]:    {msg.data}"
        self.write_to_file(line)
        print(f"Saved INFO message length: {len(msg.data)}")

    def write_to_file(self, text):
        self.file.write(text + "\n")
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CircleSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()