import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        qos_profile = QoSProfile(depth=10)
        self.sub_order = self.create_subscription(String, 'order', self.sub_order_msg, qos_profile)

    def sub_order_msg(self, msg):
        self.order_msg = msg.data
        if msg.data == 'stop':
            self.get_logger().info(msg.data)
        elif msg.data == '1':
            self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    try:
        rclpy.spin(node1)
    except KeyboardInterrupt:
        node1.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node1.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()