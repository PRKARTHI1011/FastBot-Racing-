import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('path_tracker_node')

        # --- Parameters ---
        self.lookahead_distance=2.5  # The key tuning parameter
        self.linear_speed=2.0      # Desired forward speed

        # --- Load Waypoints ---
        package_share_dir = get_package_share_directory('fastbot_racing')
        waypoints_file = '/home/user/ros2_ws/src/fastbot_racing/waypoints.csv'
        self.waypoints = self.load_waypoints(waypoints_file)
        
        # --- Subscribers and Publishers ---
        self.odom_sub = self.create_subscription(Odometry, '/fastbot_1/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/fastbot_1/cmd_vel', 10)

        self.lap_start_time = None
        self.lap_count = 0
        self.previous_closest_idx = 0

        self.get_logger().info('Pure Pursuit Node has been started.')
        

    def load_waypoints(self, filename):
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            next(reader) # Skip header
            waypoints = [[float(row[0]), float(row[1])] for row in reader]
        self.get_logger().info(f'{len(waypoints)} waypoints loaded.')
        return np.array(waypoints)

    def odom_callback(self, msg):
        # Get current robot pose
        robot_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw angle
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Find the target waypoint (look-ahead point)
        target_point = self.find_target_point(robot_pos)

        if target_point is None:
            self.stop_robot()
            return
        
        # Calculate the steering angle using Pure Pursuit logic
        # Transform the target point to the robot's coordinate frame
        dx = target_point[0] - robot_pos[0]
        dy = target_point[1] - robot_pos[1]
        target_robot_frame = np.array([
            dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw),
            dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
        ])

        # Calculate the curvature (gamma)
        L = self.lookahead_distance
        curvature = 2.0 * target_robot_frame[1] / (L ** 2)
        
        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = - curvature * twist_msg.linear.x

        self.cmd_vel_pub.publish(twist_msg)

    def find_target_point(self, robot_pos):
        lookahead_dist = self.lookahead_distance
        
        # Find the closest point on the path
        distances = np.linalg.norm(self.waypoints - robot_pos, axis=1)
        closest_idx = np.argmin(distances)

        # Search forward from the closest point to find the look-ahead point
        for i in range(closest_idx, len(self.waypoints)):
            dist_from_robot = np.linalg.norm(self.waypoints[i] - robot_pos)
            if dist_from_robot >= lookahead_dist:
                return self.waypoints[i]
        
        # If no point is far enough, return the last point
        return self.waypoints[-1]

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('End of path reached. Stopping robot.')

    # In your PurePursuitNode class
    def destroy_node(self):
        """
        Called on shutdown.
        Sends a final zero-velocity command and cleans up the node.
        """
        self.get_logger().info('Node is shutting down, sending stop command...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        super().destroy_node() # This is the original destroy_node call

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This exception is raised when you press Ctrl+C
        node.stop_robot()
    finally:
        # This block is guaranteed to be executed when the try block is exited
        # This is where we do our cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
