#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np
import math
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class DualFuzzyController(Node):
    def __init__(self):
        super().__init__('dual_fuzzy_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.robot_pose = None
        self.goal = (-8.0, 6.0)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.build_navigation_fis()
        self.build_avoidance_fis()
        self.publish_goal_marker()

        self.min_dist = float('inf')
        self.min_angle = 0.0
        self.obstacle_threshold = 0.5

    def build_navigation_fis(self):
        distance = ctrl.Antecedent(np.arange(0, 10.1, 0.1), 'distance')
        angle_error = ctrl.Antecedent(np.arange(-math.pi, math.pi, 0.01), 'angle_error')
        linear_x = ctrl.Consequent(np.arange(0, 0.6, 0.1), 'linear_x')
        angular_z = ctrl.Consequent(np.arange(-1.5, 1.6, 0.1), 'angular_z')

        distance.automf(3)

        angle_error['sharp_right'] = fuzz.trimf(angle_error.universe, [-math.pi, -1.6, -0.8])
        angle_error['right'] = fuzz.trimf(angle_error.universe, [-1.2, -0.6, -0.1])
        angle_error['zero'] = fuzz.trimf(angle_error.universe, [-0.2, 0, 0.2])
        angle_error['left'] = fuzz.trimf(angle_error.universe, [0.1, 0.6, 1.2])
        angle_error['sharp_left'] = fuzz.trimf(angle_error.universe, [0.8, 1.6, math.pi])

        linear_x['low'] = fuzz.trimf(linear_x.universe, [0, 0, 0.3])
        linear_x['medium'] = fuzz.trimf(linear_x.universe, [0.2, 0.4, 0.6])
        linear_x['high'] = fuzz.trimf(linear_x.universe, [0.4, 0.6, 0.6])

        angular_z['sharp_right'] = fuzz.trimf(angular_z.universe, [-1.5, -1.5, -0.8])
        angular_z['right'] = fuzz.trimf(angular_z.universe, [-1.0, -0.5, 0.0])
        angular_z['zero'] = fuzz.trimf(angular_z.universe, [-0.1, 0, 0.1])
        angular_z['left'] = fuzz.trimf(angular_z.universe, [0.0, 0.5, 1.0])
        angular_z['sharp_left'] = fuzz.trimf(angular_z.universe, [0.8, 1.5, 1.5])


        rules = [
            # Distance: far
            ctrl.Rule(distance['good'] & angle_error['zero'], (linear_x['high'], angular_z['zero'])),
            ctrl.Rule(distance['good'] & angle_error['left'], (linear_x['high'], angular_z['left'])),
            ctrl.Rule(distance['good'] & angle_error['right'], (linear_x['high'], angular_z['right'])),
            ctrl.Rule(distance['good'] & angle_error['sharp_left'], (linear_x['medium'], angular_z['sharp_left'])),
            ctrl.Rule(distance['good'] & angle_error['sharp_right'], (linear_x['medium'], angular_z['sharp_right'])),

            # Distance: medium
            ctrl.Rule(distance['average'] & angle_error['zero'], (linear_x['medium'], angular_z['zero'])),
            ctrl.Rule(distance['average'] & angle_error['left'], (linear_x['medium'], angular_z['left'])),
            ctrl.Rule(distance['average'] & angle_error['right'], (linear_x['medium'], angular_z['right'])),
            ctrl.Rule(distance['average'] & angle_error['sharp_left'], (linear_x['low'], angular_z['sharp_left'])),
            ctrl.Rule(distance['average'] & angle_error['sharp_right'], (linear_x['low'], angular_z['sharp_right'])),

            # Distance: low
            ctrl.Rule(distance['poor'] & angle_error['zero'], (linear_x['low'], angular_z['zero'])),
            ctrl.Rule(distance['poor'] & angle_error['left'], (linear_x['low'], angular_z['left'])),
            ctrl.Rule(distance['poor'] & angle_error['right'], (linear_x['low'], angular_z['right'])),
            ctrl.Rule(distance['poor'] & angle_error['sharp_left'], (linear_x['low'], angular_z['sharp_left'])),
            ctrl.Rule(distance['poor'] & angle_error['sharp_right'], (linear_x['low'], angular_z['sharp_right'])),
        ]


        self.nav_sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))

    def build_avoidance_fis(self):
        dist = ctrl.Antecedent(np.arange(0, 2.1, 0.1), 'dist')
        angle = ctrl.Antecedent(np.arange(-90, 91, 5), 'angle')
        lin_vel = ctrl.Consequent(np.arange(0, 0.4, 0.05), 'lin_vel')
        ang_vel = ctrl.Consequent(np.arange(-0.6, 0.61, 0.05), 'ang_vel')

        dist['close'] = fuzz.trimf(dist.universe, [0, 0, 0.7])
        dist['medium'] = fuzz.trimf(dist.universe, [0.4, 1.0, 1.6])
        dist['far'] = fuzz.trimf(dist.universe, [1.2, 2.0, 2.0])

        angle['right'] = fuzz.trimf(angle.universe, [-90, -45, 0])
        angle['center'] = fuzz.trimf(angle.universe, [-30, 0, 30])
        angle['left'] = fuzz.trimf(angle.universe, [0, 45, 90])

        lin_vel['stop'] = fuzz.trimf(lin_vel.universe, [0, 0, 0.1])
        lin_vel['slow'] = fuzz.trimf(lin_vel.universe, [0.05, 0.15, 0.25])
        lin_vel['fast'] = fuzz.trimf(lin_vel.universe, [0.2, 0.35, 0.4])

        ang_vel['left'] = fuzz.trimf(ang_vel.universe, [0.1, 0.3, 0.6])
        ang_vel['straight'] = fuzz.trimf(ang_vel.universe, [-0.1, 0, 0.1])
        ang_vel['right'] = fuzz.trimf(ang_vel.universe, [-0.6, -0.3, -0.1])


        rules = [
            # close
            ctrl.Rule(dist['close'] & angle['left'], (lin_vel['slow'], ang_vel['right'])),
            ctrl.Rule(dist['close'] & angle['right'], (lin_vel['slow'], ang_vel['left'])),
            ctrl.Rule(dist['close'] & angle['center'], (lin_vel['stop'], ang_vel['left'])),

            # medium
            ctrl.Rule(dist['medium'] & angle['left'], (lin_vel['slow'], ang_vel['right'])),
            ctrl.Rule(dist['medium'] & angle['right'], (lin_vel['slow'], ang_vel['left'])),
            ctrl.Rule(dist['medium'] & angle['center'], (lin_vel['slow'], ang_vel['straight'])),

            # far
            ctrl.Rule(dist['far'] & angle['left'], (lin_vel['fast'], ang_vel['right'])),
            ctrl.Rule(dist['far'] & angle['right'], (lin_vel['fast'], ang_vel['left'])),
            ctrl.Rule(dist['far'] & angle['center'], (lin_vel['fast'], ang_vel['straight'])),
        ]

        self.avoid_sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = np.inf
        min_idx = np.argmin(ranges)
        self.min_dist = ranges[min_idx]
        self.min_angle = math.degrees(msg.angle_min + min_idx * msg.angle_increment)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = get_yaw_from_quaternion(q)
        self.robot_pose = (x, y, yaw)

    def control_loop(self):
        if self.robot_pose is None:
            return

        if self.min_dist < self.obstacle_threshold:
            # Obstacle avoidance mode
            self.avoid_sim.input['dist'] = np.clip(self.min_dist, 0, 2)
            self.avoid_sim.input['angle'] = np.clip(self.min_angle, -90, 90)
            try:
                self.avoid_sim.compute()
                lin = float(self.avoid_sim.output['lin_vel'])
                ang = float(self.avoid_sim.output['ang_vel'])
                self.get_logger().info(f"[MODE: AVOID] Dist={self.min_dist:.2f}, Angle={self.min_angle:.1f} → V={lin:.2f}, W={ang:.2f}")
            except:
                lin, ang = 0.0, 0.0
        else:
            # Navigation mode
            dx = self.goal[0] - self.robot_pose[0]
            dy = self.goal[1] - self.robot_pose[1]
            distance = math.hypot(dx, dy)

            if distance < 0.25:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("✅ Hedefe ulaşıldı.")
                return

            angle_error = math.atan2(dy, dx) - self.robot_pose[2]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            self.nav_sim.input['distance'] = distance
            self.nav_sim.input['angle_error'] = angle_error
            try:
                self.nav_sim.compute()
                lin = float(self.nav_sim.output['linear_x'])
                ang = float(self.nav_sim.output['angular_z'])

                # Ek güvenlik: açı hatası büyükse sabit dön
                if abs(angle_error) > 0.8:
                    lin = 0.0

                self.get_logger().info(f"[MODE: NAV] Dist={distance:.2f}, Angle={angle_error:.2f} → V={lin:.2f}, W={ang:.2f}")
            except:
                lin, ang = 0.0, 0.0

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.goal[0])
        marker.pose.position.y = float(self.goal[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DualFuzzyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
