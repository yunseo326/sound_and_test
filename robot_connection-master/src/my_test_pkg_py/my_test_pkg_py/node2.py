# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile
# from std_msgs.msg import String

# class Node2(Node):
#     def __init__(self):
#         super().__init__('node2')
#         qos_profile = QoSProfile(depth=10)
#         self.node2_pub = self.create_publisher(String, 'node2_pub', qos_profile)
#         self.sub_order = self.create_subscription(String, 'order', self.sub_order_msg, qos_profile)
#         self.sub_node1 = self.create_subscription(String, 'node1_pub', self.sub_node1_msg, qos_profile)

#     def node2_pub_message(self):
#         msg = String()
#         msg.data = 'node2 talk'
#         self.node2_pub.publish(msg)

#     def sub_order_msg(self, msg):
#         self.order_msg = msg.data
#         if msg.data == 'stop':
#             self.get_logger().info('node2 stop')
#         elif msg.data == '2':
#             self.get_logger().info('node2 active')
#             self.node2_pub_message()

#     def sub_node1_msg(self, msg):
#         if self.order_msg != 'stop':
#             self.get_logger().info(f'node2 listen- {msg.data}')
#             self.node2_pub_message()


# def main(args=None):
#     rclpy.init(args=args)
#     node2 = Node2()
#     try:
#         rclpy.spin(node2)
#     except KeyboardInterrupt:
#         node2.get_logger().info('Keyboard Interrupt (SIGINT)')
#     finally:
#         node2.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()