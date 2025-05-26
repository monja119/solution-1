import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class CornRowNavigator(Node):
    def __init__(self):
        super().__init__("corn_row_navigator")

        self.cmd_pub = self.create_publisher(Twist, "/robot_base_controller/cmd_vel_unstamped", 1)
        self.image_sub = self.create_subscription(Image, "/zed2_camera_center/image_raw", self.image_callback, 1)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 1)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 1)
        self.position_sub = self.create_subscription(Odometry, "/odom", self.position_callback, 10)
        self.collision_front_left = self.create_subscription(Bool, "/front_left_collisions", self.collision_front_left_callback, 1)
        self.collision__front_right = self.create_subscription(Bool, "/front_right_collisions", self.collision_front_right_callback, 1)
        self.collision_rear_left = self.create_subscription(Bool, "/rear_left_collisions", self.collision_rear_left_callback, 1)
        self.collision_rear_right = self.create_subscription(Bool, "/rear_right_collisions", self.collision_rear_right_callback, 1)

        self.bridge = CvBridge()
        self.get_logger().info("Corn Row Navigator Started")

        self.linear_speed = 1.0
        self.angular_gain = 1.5  # Augmenter le gain pour des virages plus réactifs
        self.target_distance = 1.5

        self.no_detection_counter = 0
        self.turn_counter = 0
        self.status = "navigate"
        self.rotation = -1


        self.goal_position = (1.05, 3.81, 0.35)
        self.position = None
        self.robot_position_x = 0.0
        self.robot_position_y = 0.0
        self.robot_current_orientation = None
        self.imu_orientation = None
        self.start_yaw = None
        self.target_yaw = None
        self.lidar_ranges = None
        self.collision_detected = False
        self.rotation_timer = self.create_timer(0.05, self.rotation_callback)
        self.rotation_timer.cancel()

    def position_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.robot_position_x = position.x
        self.robot_position_y = position.y
        self.robot_current_orientation = orientation
        if self.status == "navigate" and self.start_yaw is None:
            self.start_yaw = self.quaternion_to_yaw(orientation)

    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)

    def collision_front_left_callback(self, msg):
        """Callback pour les capteurs de collision frontale gauche"""
        if msg:
            self.collision_detected = True
            # virer à 30° droite
            cmd = Twist()
            cmd.angular.z = -0.5  # Ajuster la vitesse angulaire pour un virage à droite
            cmd.linear.x = 0.1  # Réduire la vitesse linéaire pour éviter les collisions
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision frontale gauche détectée, virage à droite")
        else:
            self.collision_detected = False

    def collision_front_right_callback(self, msg):
        """Callback pour les capteurs de collision frontale droite"""
        if msg:
            self.collision_detected = True
            # virer à 30° gauche
            cmd = Twist()
            cmd.angular.z = 0.5
            cmd.linear.x = 0.1  # Réduire la vitesse linéaire pour éviter les collisions
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision frontale droite détectée, virage à gauche")
        else:
            self.collision_detected = False

    def collision_rear_left_callback(self, msg):
        """Callback pour les capteurs de collision arrière gauche"""
        if msg:
            self.collision_detected = True
            # virer à 30° gauche
            cmd = Twist()
            cmd.angular.z = 0.5
            cmd.linear.x = 0.5  # augmenter la vitesse linéaire pour éviter les collisions
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision arrière gauche détectée, accelaration et virage à gauche")
        else:
            self.collision_detected = False

    def collision_rear_right_callback(self, msg):
        """Callback pour les capteurs de collision arrière droite"""
        if msg:
            self.collision_detected = True
            # virer à 30° gauche
            cmd = Twist()
            cmd.angular.z = -0.5
            cmd.linear.x = 0.5  # augmenter la vitesse linéaire pour éviter les collisions
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision arrière droite détectée, accelaration et virage à droite")
        else:
            self.collision_detected = False



    def image_callback(self, msg):
        try:
            if self.collision_detected:
                self.get_logger().warn("Collision détectée, arrêt du traitement d'image.")
                return

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            center_offset = self.detect_corn_rows(cv_image)

            if center_offset is not None and self.status == "navigate":
                self.no_detection_counter = 0
                self.navigate_to_center(center_offset)
            else:
                self.no_detection_counter += 1
                if self.no_detection_counter > 10:
                    if self.status == "navigate":
                        is_reached = self.is_goal_reached(cv_image)
                        if is_reached:
                            self.get_logger().info("Objectif atteint, arrêt du robot.")
                            self.stop_robot()
                            return
                    self.status = "rotating"
                    self.start_turn()
                    self.search_for_rows()
                else:
                    self.stop_robot()

        except Exception as e:
            self.get_logger().error(f"Erreur traitement image: {e}")

    def detect_corn_rows(self, image):
        height, width = image.shape[:2]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        roi_height = int(height * 0.6)
        roi_y_start = height - roi_height
        green_roi = green_mask[roi_y_start:height, :]

        contours_green, _ = cv2.findContours(green_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = 1000
        valid_contours = [cnt for cnt in contours_green if cv2.contourArea(cnt) > min_area]

        if len(valid_contours) < 2:
            return None

        valid_contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)

        left_row_center = self.get_contour_center(valid_contours[0])
        right_row_center = self.get_contour_center(valid_contours[1])

        if left_row_center[0] > right_row_center[0]:
            left_row_center, right_row_center = right_row_center, left_row_center

        if abs(left_row_center[0] - right_row_center[0]) < 200:
            return None

        mid_x = int((left_row_center[0] + right_row_center[0]) / 2)
        band_width = 60
        x1 = max(0, mid_x - band_width // 2)
        x2 = min(width, mid_x + band_width // 2)
        center_band = green_roi[:, x1:x2]
        green_ratio = np.sum(center_band > 0) / center_band.size
        if green_ratio > 0.4:
            return None

        center_between_rows = (left_row_center[0] + right_row_center[0]) / 2
        image_center = width / 2
        center_offset = center_between_rows - image_center

        self.draw_debug_info(image, left_row_center, right_row_center, center_between_rows)
        return center_offset

    def is_goal_reached(self, image):
        height, width = image.shape[:2]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        goal_rgb = np.uint8([[[10, 214, 7]]])
        goal_hsv = cv2.cvtColor(goal_rgb, cv2.COLOR_RGB2HSV)[0][0]
        hue, sat, val = goal_hsv
        lower_goal = np.array([max(0, hue - 10), max(0, sat - 40), max(0, val - 40)])
        upper_goal = np.array([min(179, hue + 10), min(255, sat + 40), min(255, val + 40)])
        goal_mask = cv2.inRange(hsv, lower_goal, upper_goal)
        goal_roi = goal_mask[height//2-50:height//2+50, width//2-50:width//2+50]
        goal_ratio = np.sum(goal_roi > 0) / goal_roi.size
        if goal_ratio > 0.1:
            self.get_logger().info("Objectif atteint !")
            self.stop_robot()
            return True
        return False

    def get_contour_center(self, contour):
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        return (0, 0)

    def draw_debug_info(self, image, left_center, right_center, target_center):
        height, width = image.shape[:2]
        cv2.circle(image, left_center, 10, (0, 255, 0), -1)
        cv2.circle(image, right_center, 10, (0, 255, 0), -1)
        cv2.circle(image, (int(target_center), height//2), 15, (0, 0, 255), -1)
        cv2.line(image, (width//2, 0), (width//2, height), (255, 255, 0), 2)
        cv2.imshow("Corn Row Detection", image)
        cv2.waitKey(1)

    def navigate_to_center(self, center_offset):
        """Navigation ajustée avec LIDAR et IMU pour éviter une trajectoire trop rectiligne"""
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        normalized_offset = center_offset / 320.0
        cmd.angular.z = -normalized_offset * self.angular_gain

        # Ajustement avec LIDAR pour éviter les collisions et introduire des virages
        if self.lidar_ranges is not None:
            left_distances = self.lidar_ranges[0:45]
            right_distances = self.lidar_ranges[-45:]
            left_avg = np.mean(left_distances[np.isfinite(left_distances)])
            right_avg = np.mean(right_distances[np.isfinite(right_distances)])
            if np.isfinite(left_avg) and np.isfinite(right_avg):
                distance_diff = left_avg - right_avg
                # Réaction plus forte aux écarts
                if abs(distance_diff) > 0.1:  # Réduire le seuil pour plus de réactivité
                    cmd.angular.z += -distance_diff * 1.0  # Augmenter le coefficient
                    cmd.linear.x = max(0.1, self.linear_speed - abs(distance_diff) * 0.2)
                # Éviter les collisions proches
                if left_avg < 0.3 or right_avg < 0.3:
                    cmd.angular.z += 1.0 if left_avg < right_avg else -1.0
                    cmd.linear.x = 0.1
                    self.get_logger().warn(f"Obstacle proche détecté : gauche={left_avg:.2f}, droite={right_avg:.2f}")

        # Ajustement avec IMU pour suivre les courbes naturelles
        if self.imu_orientation is not None and self.start_yaw is not None:
            current_yaw = self.quaternion_to_yaw(self.imu_orientation)
            yaw_diff = self.normalize_angle(current_yaw - self.start_yaw)
            if abs(yaw_diff) > 0.1:  # Ajouter un petit virage si l'orientation dévie
                cmd.angular.z += -yaw_diff * 0.5
                self.get_logger().info(f"Ajustement IMU : yaw_diff={yaw_diff:.2f}, angular_z={cmd.angular.z:.2f}")

        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    def start_turn(self):
        if self.imu_orientation is not None and self.target_yaw is None:
            self.start_yaw = self.quaternion_to_yaw(self.robot_current_orientation)
            # angle d''ecart de l'orientation actuelle et l'origine (90°)
            gap_cible_angle = math.pi / 2 if self.rotation == -1 else -math.pi / 2
            gap = gap_cible_angle - self.start_yaw
            self.target_yaw = self.normalize_angle(
                self.start_yaw + math.pi
            )

            self.get_logger().info(f"Début rotation : rotation={self.rotation}, yaw initial={self.start_yaw}, yaw cible={self.target_yaw}")

    def turn_right(self):
        cmd = Twist()
        if self.imu_orientation is not None and self.target_yaw is not None:
            current_yaw = self.quaternion_to_yaw(self.imu_orientation)
            yaw_diff = self.normalize_angle(current_yaw - self.target_yaw)
            kp = 1.0
            angular_speed = -kp * yaw_diff
            angular_speed = max(-0.5, min(0.5, angular_speed))
            cmd.angular.z = angular_speed
            cmd.linear.x = max(0.1, 0.2 - abs(yaw_diff) * 0.1)
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Turn right: yaw_diff={yaw_diff:.2f}, rotation={self.rotation}")
            if abs(yaw_diff) < 0.05:
                self.get_logger().info("Rotation terminée à droite")
                self.no_detection_counter = 0
                self.turn_counter += 1
                self.status = "navigate"
                self.rotation *= -1
                self.target_yaw = None

    def turn_left(self):
        cmd = Twist()
        if self.imu_orientation is not None and self.target_yaw is not None:
            current_yaw = self.quaternion_to_yaw(self.imu_orientation)
            yaw_diff = self.normalize_angle(current_yaw - self.target_yaw)
            kp = 1.0
            angular_speed = kp * yaw_diff
            angular_speed = max(-0.5, min(0.5, angular_speed))
            cmd.angular.z = angular_speed
            cmd.linear.x = max(0.1, 0.2 - abs(yaw_diff) * 0.1)
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Turn left: yaw_diff={yaw_diff:.2f}, rotation={self.rotation}")
            if abs(yaw_diff) < 0.05:
                self.get_logger().info("Rotation terminée à gauche")
                self.no_detection_counter = 0
                self.turn_counter += 1
                self.status = "navigate"
                self.rotation *= -1
                self.target_yaw = None

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().warn("Rangées non détectées - Robot arrêté")

    def search_for_rows(self):
        self.rotation_timer.reset()

    def rotation_callback(self):
        if self.status != "rotating":
            self.rotation_timer.cancel()
            return
        if self.rotation == -1:
            self.turn_right()
        else:
            self.turn_left()

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = CornRowNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.stop