import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BallPublisherNode(Node):
    def __init__(self):
        super().__init__('ball_publisher_node')

        # Create publishers for direction and distance
        self.position_pub = self.create_publisher(Float32, '/target/position', 10)
        self.cmd_vel_pub = self.create_publisher(Float32, '/target/cmd_vel', 10)

    def publish_position(self, position: float):
        # Create and publish direction message
        position_msg = Float32()
        position_msg.data = position
        self.position_pub.publish(position_msg)
        #self.get_logger().info(f"Published position: {position}")

    def publish_distance(self, distance: float):
        # Create and publish distance message
        distance_msg = Float32()
        distance_msg.data = distance
        self.cmd_vel_pub.publish(distance_msg)
        #self.get_logger().info(f"Published distance: {distance}")
