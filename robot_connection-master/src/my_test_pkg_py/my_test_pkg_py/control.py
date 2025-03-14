"""
Example to move the robot in blind mode using ROS2 API without services.
"""
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import UInt32
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry  # 메시지 타입 변경 가능
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan  # 메시지 타입 변경 가능
#import reinforcement_model
import torch

Iteration = 100
Speed_X = 0.5
Speed_Y = 0.5
Speed_Angular = 0.5

class TestMoveBlindNoService(Node):
    def __init__(self):
        super().__init__("MoveBlindNoService")
        self.pub_action = self.create_publisher(UInt32, '/command/setAction', 10)
        self.pub_control_mode = self.create_publisher(UInt32, '/command/setControlMode', 10)

        self.pub_twist = self.create_publisher(Twist, '/mcu/command/manual_twist', 10)
        self.minimal_subscriber = MinimalSubscriber()

    def Initialize(self):
        self.get_logger().info("Setting control mode=170")
        self.pub_control_mode.publish(UInt32(data=170))
        time.sleep(0.01)
        self.pub_action.publish(UInt32(data=1)) # stand
        time.sleep(1)

        self.get_logger().info("Setting action=2")
        self.pub_action.publish(UInt32(data=2)) # walk mode

    def Forward(self):
        self.get_logger().info("Commanding forward twist")
        for i in range(Iteration):
            self.pub_twist.publish(Twist(linear=Vector3(x=Speed_X)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def Backward(self):
        self.get_logger().info("Commanding Backward twist")
        for i in range(Iteration):

            self.pub_twist.publish(Twist(linear=Vector3(x=-Speed_X)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def RightSide(self):
        self.get_logger().info("Commanding RightSide twist")
        for i in range(Iteration):
            self.pub_twist.publish(Twist(linear=Vector3(y=Speed_Y)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def LeftSide(self):
        self.get_logger().info("Commanding LeftSide twist")
        for i in range(Iteration):
            self.pub_twist.publish(Twist(linear=Vector3(y=-Speed_Y)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def Endmode(self):
        node.pub_action.publish(UInt32(data=0)) # sit
        time.sleep(5)
        node.get_logger().info("Setting control mode=180")
        node.pub_control_mode.publish(UInt32(data=180))


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,  # 수신할 메시지 타입
            '/odom',  # 구독할 토픽 이름
            self.odom_callback,  # 콜백 함수
            10  # 큐 크기
        )
        self.subscription  # 방출 방지
        self.angular =0 

    def odom_callback(self, msg):
        # 위치(Position)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # 자세(Orientation) (쿼터니언 값)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        self.angular = qz
        qw = msg.pose.pose.orientation.w

        # 속도(Twist)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z

        # 로그 출력
        self.get_logger().info(f"위치: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.get_logger().info(f"자세(쿼터니언): qx={qx:.2f}, qy={qy:.2f}, qz={qz:.2f}, qw={qw:.2f}")
        self.get_logger().info(f"속도: 선속도 x={linear_x:.2f}, 각속도 z={angular_z:.2f}")
# 라이브러리 불러오기
import torch
import torch.nn.functional as F

state_size = 96
action_size = 3

class ActorCritic(torch.nn.Module):
    def __init__(self, **kwargs):
        super(ActorCritic, self).__init__(**kwargs)
        self.d1 = torch.nn.Linear(state_size, 128)
        self.d2 = torch.nn.Linear(128, 128)
        self.pi = torch.nn.Linear(128, action_size)
        self.v = torch.nn.Linear(128, 1)
        
    def forward(self, x):
        x = F.relu(self.d1(x))
        x = F.relu(self.d2(x))
        return F.softmax(self.pi(x), dim=-1), self.v(x)
    

class LidarScan(Node):
    def __init__(self):
        super().__init__('sub_lidar')
        self.sub_order = self.create_subscription(LaserScan, '/sensor_msgs/msg/LaserScan', self.cal_action, 10)
        
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = ActorCritic().to(device)
        self.model.load_state_dict(torch.load('/pt/4action_4_mode.pt'))
        self.model.eval()

        self.output = 0

    def cal_action(self, msg):
        self.order_msg = msg.data
        self.output = self.model(self.order_msg)
        print(self.output)

if __name__=="__main__":
    rclpy.init(args=None)
    node = TestMoveBlindNoService()
    minimal_subscriber = MinimalSubscriber()
    lidar = LidarScan()
    
    output = rclpy.spin(lidar)
    node.Initialize()
    print("forward : 1 \n backward : 2 \n right : 3 \n left : 6 \n end : 7 \n ")
    
    In = 0
    while True:
        if lidar.output == "hihi":
            print(output)
        
        if In == "1":
            node.Forward()
            rclpy.spin_once(minimal_subscriber)
        elif In == "2":
            node.Backward()
            rclpy.spin_once(minimal_subscriber)
        elif In == "3":
            node.RightSide()
            rclpy.spin_once(minimal_subscriber)
        elif In == "4":
            node.LeftSide()
            rclpy.spin_once(minimal_subscriber)
        
        elif In =="7":
            node.Endmode()
            break

    node.destroy_node()
    rclpy.shutdown()