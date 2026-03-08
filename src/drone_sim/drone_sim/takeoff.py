import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random


class DroneControl(Node):

    def __init__(self):
        super().__init__('drone_control')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.3, self.control_loop)

        self.altitude = 0
        self.target_altitude = 60

        self.boundary = 80
        self.distance_travelled = 0

        self.phase = "takeoff"
        self.turn_steps = 0

    def control_loop(self):

        msg = Twist()

        # ---------- TAKEOFF ----------
        if self.phase == "takeoff":

            if self.altitude < self.target_altitude:
                msg.linear.z = 1.5
                self.altitude += 1
            else:
                self.phase = "explore"
                self.get_logger().info("Reached altitude")

        # ---------- EXPLORATION ----------
        elif self.phase == "explore":

            msg.linear.x = random.uniform(3.0, 5.0)
            msg.angular.z = random.uniform(-0.2, 0.2)

            self.distance_travelled += 1

            if self.distance_travelled > self.boundary:
                self.phase = "turn"
                self.turn_steps = random.randint(8, 14)
                self.get_logger().info("Boundary reached, turning")

        # ---------- TURNING ----------
        elif self.phase == "turn":

            msg.linear.x = 0.0
            msg.angular.z = 1.5

            self.turn_steps -= 1

            if self.turn_steps <= 0:
                self.phase = "explore"
                self.distance_travelled = 0
                self.get_logger().info("Turn complete")

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DroneControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
