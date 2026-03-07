import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DroneControl(Node):

    def __init__(self):
        super().__init__('drone_control')
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(0.5,self.move)

    def move(self):
        msg = Twist()
        msg.linear.z = 1.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
