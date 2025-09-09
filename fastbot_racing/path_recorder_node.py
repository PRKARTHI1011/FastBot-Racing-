import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv

class PathRecorderNode(Node):
    def __init__(self):
        super().__init__('path_recorder_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/fastbot_1/odom',
            self.odom_callback,
            10)
        # Change the file path to your desired location
        self.csv_file = open('waypoints.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y'])
        self.get_logger().info("Path Recorder started. Drive the robot manually to record waypoints.")
        self.get_logger().info("Press Ctrl+C to stop recording.")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        x, y = position.x, position.y
        self.csv_writer.writerow([x, y])
        self.get_logger().info(f'Logged waypoint: ({x:.2f}, {y:.2f})', throttle_duration_sec=1.0)
        
    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info("Waypoints saved to waypoints.csv")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()