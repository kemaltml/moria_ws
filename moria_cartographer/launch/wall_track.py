#!/usr/bin/env python3

import os
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from transforms3d.euler import euler2quat, quat2euler
from geometry_msgs.msg import PoseStamped, Point, TransformStamped, Twist


class WallTrackNode(Node):
    def __init__(self):
        super().__init__('wall_track')

        # Subscriptions
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.timer = self.create_timer(0.25, self.get_robot_position)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10 )
        
    
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parameters
        self.min_front_distance = 1.5
        self.min_right_distance = 0.5
        #self.map_resolution = None
        #self.map_origin = None
        self.vertical_appr = 0
        self.arr = []
        self.start_flag = 0
        self.err = 0
        self.wall_track = 0
        self.prev_error = 0.0
        self.integral = 0.0
        self.kp = 1
        self.ki = 0.01
        self.kd = 0.1
        self.map_call = 0
        self.value = 0
        self.directions = [0, 1.57, 3.14, -1.57 ]
        self.steer_ctl = 0
        
        # Log
        self.get_logger().info("WALL TRACK NODE INITIALIZED")

        # Definitions
        self.twist_msg = Twist()

    def map_callback(self, msg):
        print("MAP CALLBACK")
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.get_robot_position()
        self.map_call = 1 # lidar_callback will start after map_callback

        
        

    def lidar_callback(self, msg):
        print("LIDAR CALLBACK")
        if self.map_call == 1:
            ranges = msg.ranges
            for i in range(len(ranges)):
                if ranges[i] == float('inf'):
                    ranges[i] = 100.0

            self.front = round(ranges[89],2)
            self.right = round(ranges[0],2)
            self.left = round(ranges[179],2)
            self.d45 = round(ranges[45],2)
            self.d30 = round(ranges[30],2)
            self.d10 = round(ranges[10],2)

            self.min_range = min(ranges)
            self.min_index = ranges.index(self.min_range)
            self.min_range = round(self.min_range, 2)

            print("---------------------------------------------------------------")
            print(f'RIGHT: {self.right}')
            print(f'FRONT: {self.front}')
            print(f'LEFT:  {self.left}')
            print(f'MIN RANGE: {self.min_range} @ {self.min_index}')

            self.yaw = round(self.yaw,2)
            print(f"YAW: {self.yaw}")

            if 0 < abs(self.yaw) < 0.3:
                self.value = 0
            elif 1.4 < self.yaw < 1.8:
                self.value = 1
            elif 2.9 < abs(self.yaw) < 3.14:
                self.value = 2
            elif -1.8 < self.yaw < -1.4:
                self.value = 3

            
            self.direction = self.directions[self.value%4]
            print(f"DIRECTION: {self.direction}, VALUE: {self.value}")
            
            
            # Checking right of the robot, 
            # If wall stop appearing robot will turn right. 
            # For that always compare lidars current value with previus value
            self.arr.append(ranges[45])
            if len(self.arr) == 3:
                del self.arr[0]
            print(self.arr)
            self.start()
        else:
            print("WAITING MAP_CALLBACK")

    def start(self):
        if self.start_flag == 0: # In the beginning map wont be as expected so we will move and turn robot for better and bigger start map
            self.align_wall() 
        else:
            if self.front < 0.7:
                print("front is close")
                self.steer()
            else:
                self.move()

    def align_wall(self):
        #os.system('clear')
        twist_msg = Twist()
        print("ALIGNING WALL")
        if self.vertical_appr == 0 and self.min_index > 30:
            
            print("VERTICAL APPROACHING")
            if self.front > 1.5:
                print("V SPEED IS 0.35")
                twist_msg.linear.x = 0.35
                twist_msg.angular.z = 0.0
            elif 0.5 < self.front < 1.5:
                print("V SPEED IS 0.2")
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0.0
            else:
                print("DISTANCE IS AVAILABLE")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.pid_compute(90, -self.min_index)
                self.vertical_appr = 1
            self.twist_pub.publish(self.twist_msg)
        else:
            #if not round(self.yaw,2) == 3.14:
            if not self.min_index < 5:
                print("TURNING ROBOT FOR BE PARALLEL")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.2
            else:
                print("ALIGN PRECEDURE IS DONE...")
                self.start_flag = 1
            self.twist_pub.publish(twist_msg)
        self.twist_pub.publish(twist_msg)


    def steer(self):
        print("steer() called")
        if self.steer_ctl == 1 and self.front < 2.0:
            print("conditions for steer are OK")
            if self.d30 > 1.5:
                print("steering right")
                self.twist_msg.linear.x = 0.1
                self.twist_msg.angular.z = -0.3
                self.twist_pub.publish(self.twist_msg)
            else:
                print("steering left")
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.3
                self.twist_pub.publish(self.twist_msg)
    

    def move(self):
        #os.system('clear')
        print("moving")
        print("following wall")
        print(f"index 45: {self.d45} PID")

        if self.d30 > 1.5:
            self.steer_ctl = 0
        else: 
            self.steer_ctl = 1

        angular = self.pid_compute(0.45, self.d30)
        self.twist_msg.angular.z = angular

        # Checking for If robot will turn fast or slow. 
        # If robot will turning fast it will be slow in linear so change of position will be minimum.
        # And it will make easy to apply PID and set the position about wall
        if abs(angular) > 0.3:
            self.twist_msg.linear.x = 0.2
        else:
            self.twist_msg.linear.x = 0.3

        self.twist_pub.publish(self.twist_msg)

    def pid_compute(self, target, current): # I is not using right know. Will add time 
        print("pid calculating...")
        error = target - current 
        self.integral += error 
        derivative = error - self.prev_error
        p = round(self.kp * error,4)
        i = round(self.ki * self.integral,4)
        d = round(self.kd * derivative,4)
        print(f"P: {p}\nI: {i}\nD: {d}")

        if i > 1.0:
            i = 0.5
        if i < -1.0:
            i = -0.5
        
        output = p + d
        output = round(output, 4)
        print(f"error: {error}    output: {output}")


        if output > 0.4:
            output = 0.4

        if output < -0.4:
            output = -0.4

        self.prev_error = error
        print(f"P: {p}\nI: {i}\nD: {d}")
        print(f"error: {error}    output: {output}")
        print(f"RETURNING PID {float(output)}")
        return float(output)
        

    def get_robot_position(self):
        #os.system('clear')
        #self.get_logger().info("GETTING ROBOT POSITION")
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.position = transform.transform.translation
            self.orientation = transform.transform.rotation
            
            # Round the position values to 1 decimal place
            self.position.x = round(self.position.x, 1)
            self.position.y = round(self.position.y, 1)
            self.position.z = round(self.position.z, 1)

            # Round the orientation values to 1 decimal place
            self.orientation.x = round(self.orientation.x, 1)
            self.orientation.y = round(self.orientation.y, 1)
            self.orientation.z = round(self.orientation.z, 1)
            self.orientation.w = round(self.orientation.w, 1)

            self.quaternion_to_yaw(
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
                self.orientation.w
            )

                
            #self.get_logger().info(f"Robot position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
            #self.get_logger().info(f"Robot orientation (quaternion): z={self.orientation.z}, w={self.orientation.w}")
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")

    def goal_pose(self, x, y, direction):
        #os.system('clear')
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        x,y = self.check_available(int(x),int(y),self.map_data)
        goal_pose.pose.position.x = x #* self.map_resolution + self.map_origin.position.x
        goal_pose.pose.position.y = y #* self.map_resolution + self.map_origin.position.y

        
        if direction == 'E':
            goal_pose.pose.orientation.z = 0.00
            goal_pose.pose.orientation.w = 1.00
        elif direction == 'N':
            goal_pose.pose.orientation.z = 0.70
            goal_pose.pose.orientation.w = 0.70
        elif direction == 'W':
            goal_pose.pose.orientation.z = 1.00
            goal_pose.pose.orientation.w = 0.00
        elif direction == 'S':
            goal_pose.pose.orientation.z = 0.70
            goal_pose.pose.orientation.w = -0.70

        print(f"HEADING {direction}")
        
        self.goal_pub.publish(goal_pose)
        self.get_logger().info(f"Publishing Goal: x={x}, y={y}, z={goal_pose.pose.orientation.z}, w={goal_pose.pose.orientation.w}")

    import numpy as np

    def check_available(self,x, y, map_data, max_radius=10):
        #os.system('clear')
        """
        Check if the given location (x, y) is free and find the nearest valid location if not.

        Args:
            x (int): Target x-coordinate in grid.
            y (int): Target y-coordinate in grid.
            map_data (numpy.ndarray): 2D array representing the occupancy grid.
            max_radius (int): Maximum radius to search for a valid location.

        Returns:
            tuple: (x, y) of the nearest valid location, or None if no valid location is found.
        """

        # Check if the target location is free
        if map_data[x, y] == 0:
            print("LOCATION IS AVAILABLE")
            return x, y

        print("LOCATION NOT AVAILABLE, CALCULATING NEW LOCATION")

        # Search for the nearest free cell within the max radius
        for radius in range(1, max_radius + 1):
            # Get the bounding box for the current radius
            x_min = max(0, x - radius)
            x_max = min(self.map_height, x + radius + 1)
            y_min = max(0, y - radius)
            y_max = min(self.map_width, y + radius + 1)

            # Extract the subregion of the map
            subregion = map_data[x_min:x_max, y_min:y_max]

            # Find free cells in the subregion
            free_cells = np.argwhere(subregion == 0)
            if free_cells.size > 0:
                # Convert local coordinates to global coordinates
                nearest_cell = free_cells[0]  # Take the first free cell
                x = x_min + nearest_cell[0]
                y = y_min + nearest_cell[1]
                return float(x),float(y)

        print("NO VALID LOCATION FOUND WITHIN MAX RADIUS")
        return None


    def quaternion_to_yaw(self, x, y, z, w):
        self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def calculate_coordinate(self, angle, distance):
        ##os.system('clear')
        self.get_logger().info("CALCULATING COORDINATE TO GO")
        angle = math.radians(angle) + self.yaw 
        local_x = distance * math.cos(angle)
        local_y = distance * math.sin(angle)
        self.get_logger().info("CALCULATED COORDINATES")
        self.get_logger().info(f"local_x: {local_x}  local_y: {local_y}")

        self.map_x = self.position.x + (local_x * math.cos(self.yaw) - local_y * math.sin(self.yaw))
        self.map_x = round(self.map_x, 2)
        self.map_y = self.position.y + (local_x + math.sin(self.yaw) + local_y * math.cos(self.yaw))
        self.map_y = round(self.map_y, 2)
        self.get_logger().info(f"map_x: {self.map_x}     map_y: {self.map_y}")

def main(args=None):
    rclpy.init(args=args)
    node = WallTrackNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_looger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
