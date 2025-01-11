#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration')

        # Parameters
        self.min_frontier_distance = 0.5  # Minimum distance to consider a frontier valid
        self.map_resolution = None       # To be updated dynamically from the map
        self.map_origin = None           # Origin of the map
        self.visited_frontiers = []      # List of visited frontiers

        # Subscriptions and Publishers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.frontier_pub = self.create_publisher(Marker, '/visualized_frontiers', 10)
        self.selected_frontier_pub = self.create_publisher(Marker, '/selected_frontier', 10)

        self.get_logger().info("Improved Frontier Exploration Node Initialized")

    def map_callback(self, msg):
        # Update map resolution and origin
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

        # Convert map data to a 2D numpy array
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # Detect frontiers
        frontiers = self.detect_frontiers(map_data)

        if not frontiers:
            self.get_logger().info("No frontiers detected")
            return

        # Visualize frontiers in RViz
        self.visualize_frontiers(frontiers, msg.info)

        # Select the best frontier
        best_frontier = self.select_best_frontier(frontiers, msg.info, map_data)
        if best_frontier:
            self.publish_goal(best_frontier, msg.info)
            self.visualize_selected_frontier(best_frontier, msg.info)
            self.visited_frontiers.append(best_frontier)
        else:
            self.get_logger().info("No valid frontiers found for navigation")

    def detect_frontiers(self, map_data):
        """Detect frontier regions by finding unknown cells adjacent to free cells."""
        frontiers = []
        for i in range(1, map_data.shape[0] - 1):
            for j in range(1, map_data.shape[1] - 1):
                if map_data[i, j] == -1:  # Unknown cell
                    # Check if adjacent cells are free (value 0)
                    if np.any(map_data[i - 1:i + 2, j - 1:j + 2] == 0):
                        frontiers.append((i, j))
        return frontiers

    def select_best_frontier(self, frontiers, map_info, map_data):
        """Select the best frontier based on distance, information gain, and context."""
        best_score = float('-inf')
        best_frontier = None
        robot_x = self.map_origin.position.x
        robot_y = self.map_origin.position.y

        for frontier in frontiers:
            x, y = frontier
            frontier_x = x * map_info.resolution + map_info.origin.position.x
            frontier_y = y * map_info.resolution + map_info.origin.position.y

            # Check if this frontier has already been visited
            if self.is_visited(frontier_x, frontier_y):
                continue

            # Calculate distance to the robot
            distance = np.sqrt((frontier_x - robot_x)**2 + (frontier_y - robot_y)**2)

            # Information gain: Count unknown cells around the frontier
            info_gain = self.calculate_information_gain(x, y, map_info, map_data)

            # Contextual priority: Prefer frontiers in the middle of open areas
            spatial_context_score = self.calculate_spatial_context(frontier, map_info, map_data)

            # Combine scores into a weighted sum
            score = -distance + 2.0 * info_gain + 1.5 * spatial_context_score  # Adjust weights as needed

            if score > best_score:
                best_score = score
                best_frontier = frontier

        return best_frontier

    def calculate_information_gain(self, x, y, map_info, map_data):
        """Calculate the number of unknown cells around the frontier."""
        unknown_count = 0
        height, width = map_info.height, map_info.width

        for i in range(-3, 4):  # Radius of 3 cells
            for j in range(-3, 4):
                nx, ny = x + i, y + j
                if 0 <= nx < height and 0 <= ny < width:
                    if map_data[nx, ny] == -1:  # Unknown cell
                        unknown_count += 1
        return unknown_count

    def calculate_spatial_context(self, frontier, map_info, map_data):
        """Evaluate the frontier's location within an open area."""
        x, y = frontier
        left = right = up = down = 0
        height, width = map_info.height, map_info.width

        for i in range(1, 5):  # Check up to 5 cells in each direction
            if x - i >= 0 and map_data[x - i, y] == 0:
                up += 1
            if x + i < height and map_data[x + i, y] == 0:
                down += 1
            if y - i >= 0 and map_data[x, y - i] == 0:
                left += 1
            if y + i < width and map_data[x, y + i] == 0:
                right += 1

        # Prefer frontiers in the middle of symmetric open areas
        return min(up, down) + min(left, right)

    def is_visited(self, frontier_x, frontier_y):
        """Check if a frontier has already been visited."""
        for visited_x, visited_y in self.visited_frontiers:
            if np.sqrt((visited_x - frontier_x)**2 + (visited_y - frontier_y)**2) < 0.5:  # 0.5m threshold
                return True
        return False

    def publish_goal(self, frontier, map_info):
        """Publish the selected frontier as a goal for Nav2."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = frontier[1] * map_info.resolution + map_info.origin.position.x
        goal.pose.position.y = frontier[0] * map_info.resolution + map_info.origin.position.y
        goal.pose.orientation.w = 1.0  # Facing forward

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published Goal: x={goal.pose.position.x}, y={goal.pose.position.y}")

    def visualize_frontiers(self, frontiers, map_info):
        """Visualize detected frontiers in blue in RViz."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Blue for frontiers
        marker.color.a = 1.0

        for frontier in frontiers:
            point = Point()
            point.x = frontier[1] * map_info.resolution + map_info.origin.position.x
            point.y = frontier[0] * map_info.resolution + map_info.origin.position.y
            point.z = 0.0
            marker.points.append(point)

        self.frontier_pub.publish(marker)
        self.get_logger().info(f"Visualized {len(frontiers)} frontiers in blue")

    def visualize_selected_frontier(self, frontier, map_info):
        """Visualize the selected frontier in green in RViz."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green for the selected frontier
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = frontier[1] * map_info.resolution + map_info.origin.position.x
        marker.pose.position.y = frontier[0] * map_info.resolution + map_info.origin.position.y
        marker.pose.position.z = 0.0

        self.selected_frontier_pub.publish(marker)
        self.get_logger().info("Visualized selected frontier in green")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt - Shutting Down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
