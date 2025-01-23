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
from std_msgs.msg import Float64
import time


class WallTrackNode(Node):
    def __init__(self):
        """
        Initialize the WallTrackNode class.

        This function initializes the ROS node, subscriptions, publishers, parameters, and other necessary components for the wall tracking algorithm.

        Parameters:
        None

        Returns:
        None
        """
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
        self.start_flag = 0
        self.err = 0
        self.prev_error = 0.0
        self.integral = 0.0
        self.kp = 1
        self.ki = 0.01
        self.kd = 0.1
        self.map_call = 0
        self.value = 0
        self.directions = [0, 1.57, 3.14, -1.57 ]
        self.front_wall = 0

        # Log
        self.get_logger().info("WALL TRACK NODE INITIALIZED")

        # Definitions
        self.twist_msg = Twist()

    def map_callback(self, msg):
        """
        Callback function for processing the received map message.

        This function is called when a new map message is received. It extracts
        and stores relevant map information, reshapes the map data, updates the
        robot's position, and sets a flag to indicate that the map has been received.

        Parameters:
        msg (OccupancyGrid): The received map message containing map information.

        Returns:
        None
        """
        print("MAP CALLBACK")
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.get_robot_position()
        self.map_call = 1 # lidar_callback will start after map_callback




    def lidar_callback(self, msg):
        """
        Process the LiDAR scan data and update robot's state.

        This function is called when new LiDAR scan data is received. It processes
        the scan data, updates various distance measurements, calculates the minimum
        range, determines the robot's orientation, and initiates the start procedure.

        Parameters:
        msg (sensor_msgs.msg.LaserScan): The LiDAR scan message containing range data.

        Returns:
        None
        """
        print("LIDAR CALLBACK")
        if self.map_call == 1:
            self.ranges = msg.ranges
            for i in range(len(self.ranges)):
                if self.ranges[i] == float('inf'):
                    self.ranges[i] = 100.0

            self.prev_time = None
            self.prev_error = 0.0
            self.integral = 0.0

            self.front = round(self.ranges[89],2)
            self.right = round(self.ranges[0],2)
            self.left = round(self.ranges[179],2)
            self.d45 = round(self.ranges[45],2)
            self.d30 = round(self.ranges[30],2)
            self.d20 = round(self.ranges[20],2)
            self.d15 = round(self.ranges[15],2)
            self.d10 = round(self.ranges[10],2)

            self.min_range = min(self.ranges)
            self.min_index = self.ranges.index(self.min_range)
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

            self.start()
        else:
            print("WAITING MAP_CALLBACK")


    def start(self):
        if self.start_flag == 0: # In the beginning map wont be as expected so we will move and turn robot for better and bigger start map
            self.align_wall()
        else:
            if self.front<0.7:
                print("front is close")
                if abs(self.yaw) < 0.1 and not self.yaw == 1.57:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.05
                    twist_msg.angular.z = 0.2
                    self.twist_pub.publish(twist_msg)
                else:
                    self.steer()
            else:
                self.move()

    def align_wall(self):
        #os.system('clear')
        twist_msg = Twist()
        print("ALIGNING WALL")
        if self.vertical_appr == 0 and (self.min_index > 30 or self.min_range > 2.0):
            if self.front_wall == 0:
                if not 85< self.min_index< 95:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.2
                    self.twist_pub.publish(twist_msg)
                else:
                    self.front_wall = 1

            else:
                print("VERTICAL APPROACHING")
                if self.min_range> 1.5:
                    print("V SPEED IS 0.35")
                    twist_msg.linear.x = 0.35
                    twist_msg.angular.z = 0.0
                elif 1.0< self.min_range < 1.5:
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
        if self.d30 > 2:
            print("steering right")
            self.twist_msg.linear.x = 0.1
            self.twist_msg.angular.z = -0.3
            self.twist_pub.publish(self.twist_msg)
        else:
            print("steering left")
            self.twist_msg.linear.x = 0.05
            self.twist_msg.angular.z = 0.3
            self.twist_pub.publish(self.twist_msg)
            if self.ranges[80] < 4.0:
                print("steering left")


    def move(self):
        #os.system('clear')
        print("moving")
        print("following wall")



        if self.d45 < 2.5:
            angular = self.pid_compute(1.0, self.d45)
            print(f"index 45: {self.d45} PID")
        elif self.d10 < 1.5:
            angular = self.pid_compute(0.85, self.d10)
            print(f"index 10: {self.d10} PID")
        elif self.d45 > 2.5 and self.d10 > 2.5 and self.d30 > 2.5:
            angular = -0.3
        else:
            angular = self.pid_compute(1.0, self.d45)




        self.twist_msg.angular.z = angular

        # Checking for If robot will turn fast or slow.
        # If robot will turning fast it will be slow in linear so change of position will be minimum.
        # And it will make easy to apply PID and set the position about wall
        if abs(angular) > 0.35:
            self.twist_msg.linear.x = 0.2
        else:
            self.twist_msg.linear.x = 0.2

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


        if output > 0.5:
            output = 0.5

        if output < -0.4:
            output = -0.3

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



    def quaternion_to_yaw(self, x, y, z, w):
        self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))



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

