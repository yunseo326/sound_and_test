import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from std_msgs.msg import UInt32
from geometry_msgs.msg import Vector3, Twist
from std_srvs.srv import Empty

Iteration = 100
Speed_X = 0.5
Speed_Y = 0.5
Speed_Angular = 0.5

# 노드 이름 angle_node
# 받아야하는 토픽 이름 angle
# 토픽 타입

class Node1(Node):
    def __init__(self):
        super().__init__('angle_node')
        qos_profile = QoSProfile(depth=10)
        self.sub_angle = self.create_subscription(String, 'angle', self.turn, qos_profile)

        self.pub_action = self.create_publisher(UInt32, '/command/setAction', 10)
        self.pub_control_mode = self.create_publisher(UInt32, '/command/setControlMode', 10)
        self.pub_twist = self.create_publisher(Twist, '/mcu/command/manual_twist', 10)

    # 초기화 함수
    def Initialize(self):
        self.get_logger().info("Setting control mode=170")
        self.pub_control_mode.publish(UInt32(data=170))
        time.sleep(0.01)
        self.pub_action.publish(UInt32(data=1)) # stand
        time.sleep(1)

        self.get_logger().info("Setting action=2")
        self.pub_action.publish(UInt32(data=2)) # walk mode

    # 이부분 돌아가는 명령어 뭔지 맞춰야함
    def turn(self):
        self.get_logger().info("Detecting enemy Turn")
        for i in range(Iteration):
            self.pub_twist.publish(Twist(linear=Vector3(x=Speed_X)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    node1.Initialize()
    try:
        rclpy.spin(node1)
    except KeyboardInterrupt:
        node1.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node1.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()