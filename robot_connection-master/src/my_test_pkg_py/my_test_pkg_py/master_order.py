import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class MasterOrder(Node):
    def __init__(self):
        super().__init__('master_order')
        qos_profile = QoSProfile(depth=10)
        self.master_order = self.create_publisher(String, 'order', qos_profile)

    def publish_order(self):
        msg = String()
        msg.data = self.order
        self.master_order.publish(msg)
        self.get_logger().info(f'Order message: {msg.data}')

    def input_order(self):
        self.order = input("Order: ")


def main(args=None):
    rclpy.init(args=args)
    master_order = MasterOrder()

    while True:
        try:
            master_order.input_order()
            if master_order.order == '1':
                master_order.publish_order()
            elif master_order.order == 'stop':
                master_order.get_logger().info('Order stop')
                master_order.publish_order()
            else:
                master_order.get_logger().info('Try again')
                continue
        except KeyboardInterrupt:
            master_order.get_logger().info('Keyboard Interrupt (SIGINT)')
            break

    master_order.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()