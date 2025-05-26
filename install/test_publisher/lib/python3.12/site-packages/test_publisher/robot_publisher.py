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
        self.angular_gain = 1.5
        self.target_distance = 1.5

        self.no_detection_counter = 0
        self.turn_counter = 0
        self.status = "navigate"
        self.rotation_direction = 1  # 1 pour droite, -1 pour gauche
        self.turn_state = "start"  # "start", "turning", "moving_forward", "final_turn"

        self.goal_position = (1.05, 3.81, 0.35)
        self.position = None
        self.robot_position_x = 0.0
        self.robot_position_y = 0.0
        self.robot_current_orientation = None
        self.imu_orientation = None
        self.start_yaw = None
        self.target_yaw = None
        self.initial_yaw = None  # Orientation initiale pour revenir après rotation
        self.lidar_ranges = None
        self.collision_detected = False
        
        # Variables pour gérer la séquence de rotation complète
        self.turn_phase = 0  # 0: premier virage, 1: avance, 2: deuxième virage
        self.forward_distance_target = 2.0  # Distance à parcourir en ligne droite
        self.forward_start_position = None
        
        self.rotation_timer = self.create_timer(0.05, self.rotation_callback)
        self.rotation_timer.cancel()

    def position_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.robot_position_x = position.x
        self.robot_position_y = position.y
        self.robot_current_orientation = orientation
        if self.status == "navigate" and self.initial_yaw is None:
            self.initial_yaw = self.quaternion_to_yaw(orientation)

    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)

    def collision_front_left_callback(self, msg):
        if msg.data:
            self.collision_detected = True
            cmd = Twist()
            cmd.angular.z = -0.5
            cmd.linear.x = 0.1
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision frontale gauche détectée, virage à droite")
        else:
            self.collision_detected = False

    def collision_front_right_callback(self, msg):
        if msg.data:
            self.collision_detected = True
            cmd = Twist()
            cmd.angular.z = 0.5
            cmd.linear.x = 0.1
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision frontale droite détectée, virage à gauche")
        else:
            self.collision_detected = False

    def collision_rear_left_callback(self, msg):
        if msg.data:
            self.collision_detected = True
            cmd = Twist()
            cmd.angular.z = 0.5
            cmd.linear.x = 0.5
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision arrière gauche détectée, acceleration et virage à gauche")
        else:
            self.collision_detected = False

    def collision_rear_right_callback(self, msg):
        if msg.data:
            self.collision_detected = True
            cmd = Twist()
            cmd.angular.z = -0.5
            cmd.linear.x = 0.5
            self.cmd_pub.publish(cmd)
            self.get_logger().warn("Collision arrière droite détectée, acceleration et virage à droite")
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
                    self.status = "u_turn"
                    self.start_u_turn()
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
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        normalized_offset = center_offset / 320.0
        cmd.angular.z = -normalized_offset * self.angular_gain

        if self.lidar_ranges is not None:
            left_distances = self.lidar_ranges[0:45]
            right_distances = self.lidar_ranges[-45:]
            left_avg = np.mean(left_distances[np.isfinite(left_distances)])
            right_avg = np.mean(right_distances[np.isfinite(right_distances)])
            if np.isfinite(left_avg) and np.isfinite(right_avg):
                distance_diff = left_avg - right_avg
                if abs(distance_diff) > 0.1:
                    cmd.angular.z += -distance_diff * 1.0
                    cmd.linear.x = max(0.1, self.linear_speed - abs(distance_diff) * 0.2)
                if left_avg < 0.3 or right_avg < 0.3:
                    cmd.angular.z += 1.0 if left_avg < right_avg else -1.0
                    cmd.linear.x = 0.1
                    self.get_logger().warn(f"Obstacle proche détecté : gauche={left_avg:.2f}, droite={right_avg:.2f}")

        if self.imu_orientation is not None and self.initial_yaw is not None:
            current_yaw = self.quaternion_to_yaw(self.imu_orientation)
            yaw_diff = self.normalize_angle(current_yaw - self.initial_yaw)
            if abs(yaw_diff) > 0.1:
                cmd.angular.z += -yaw_diff * 0.5
                self.get_logger().info(f"Ajustement IMU : yaw_diff={yaw_diff:.2f}, angular_z={cmd.angular.z:.2f}")

        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    def start_u_turn(self):
        """Démarre une séquence de demi-tour complète"""
        if self.robot_current_orientation is not None:
            self.start_yaw = self.quaternion_to_yaw(self.robot_current_orientation)
            self.turn_phase = 0  # Phase 0: premier virage de 90°
            
            # Déterminer la direction de rotation (alternance)
            self.rotation_direction = 1 if self.turn_counter % 2 == 0 else -1
            
            # Premier virage : 90° vers la droite ou la gauche
            self.target_yaw = self.normalize_angle(
                self.start_yaw + (math.pi / 2) * self.rotation_direction
            )
            
            self.get_logger().info(f"Début demi-tour : phase={self.turn_phase}, direction={self.rotation_direction}, yaw_cible={self.target_yaw}")
            self.rotation_timer.reset()

    def execute_u_turn(self):
        """Exécute la séquence complète de demi-tour"""
        if self.imu_orientation is None:
            return
            
        current_yaw = self.quaternion_to_yaw(self.imu_orientation)
        cmd = Twist()
        
        if self.turn_phase == 0:  # Premier virage (90°)
            yaw_diff = self.normalize_angle(self.target_yaw - current_yaw)
            
            if abs(yaw_diff) > 0.05:
                # Continue le virage
                angular_speed = 0.5 * self.rotation_direction
                cmd.angular.z = angular_speed
                cmd.linear.x = 0.1  # Vitesse réduite pendant le virage
                self.get_logger().info(f"Phase 0 - Virage: yaw_diff={yaw_diff:.2f}, angular_z={cmd.angular.z:.2f}")
            else:
                # Premier virage terminé, passer à la phase suivante
                self.turn_phase = 1
                self.forward_start_position = (self.robot_position_x, self.robot_position_y)
                self.get_logger().info("Phase 0 terminée, passage à la phase 1 (avance)")
                
        elif self.turn_phase == 1:  # Avancer en ligne droite
            if self.forward_start_position is not None:
                distance_traveled = math.sqrt(
                    (self.robot_position_x - self.forward_start_position[0])**2 + 
                    (self.robot_position_y - self.forward_start_position[1])**2
                )
                
                if distance_traveled < self.forward_distance_target:
                    # Continue d'avancer
                    cmd.linear.x = 0.5
                    cmd.angular.z = 0.0
                    self.get_logger().info(f"Phase 1 - Avance: distance={distance_traveled:.2f}/{self.forward_distance_target}")
                else:
                    # Distance parcourue, passer au deuxième virage
                    self.turn_phase = 2
                    self.target_yaw = self.normalize_angle(
                        current_yaw + (math.pi / 2) * self.rotation_direction
                    )
                    self.get_logger().info("Phase 1 terminée, passage à la phase 2 (deuxième virage)")
                    
        elif self.turn_phase == 2:  # Deuxième virage (90°)
            yaw_diff = self.normalize_angle(self.target_yaw - current_yaw)
            
            if abs(yaw_diff) > 0.05:
                # Continue le virage
                angular_speed = 0.5 * self.rotation_direction
                cmd.angular.z = angular_speed
                cmd.linear.x = 0.1
                self.get_logger().info(f"Phase 2 - Virage final: yaw_diff={yaw_diff:.2f}")
            else:
                # Demi-tour complet terminé
                self.get_logger().info("Demi-tour terminé, retour à la navigation")
                self.complete_u_turn()
                return
        
        self.cmd_pub.publish(cmd)

    def complete_u_turn(self):
        """Termine la séquence de demi-tour et retourne à la navigation"""
        self.status = "navigate"
        self.turn_counter += 1
        self.no_detection_counter = 0
        self.turn_phase = 0
        self.target_yaw = None
        self.forward_start_position = None
        self.rotation_timer.cancel()
        
        # Mettre à jour l'orientation de référence
        if self.robot_current_orientation is not None:
            self.initial_yaw = self.quaternion_to_yaw(self.robot_current_orientation)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().warn("Robot arrêté")

    def rotation_callback(self):
        if self.status == "u_turn":
            self.execute_u_turn()
        else:
            self.rotation_timer.cancel()

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
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()