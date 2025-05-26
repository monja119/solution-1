import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class CornRowNavigator(Node):
    def __init__(self):
        super().__init__("corn_row_navigator")
        # Publishers et Subscribers
        self.cmd_pub = self.create_publisher(Twist, "/robot_base_controller/cmd_vel_unstamped", 10)
        self.image_sub = self.create_subscription(Image, "/zed2_camera_center/image_raw", self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.position_callback, 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Démarrage du navigateur de rangées de maïs")

        # Paramètres
        self.linear_speed = 5.0
        self.status = "navigate"
        self.rotation = -1  # -1 pour droite, 1 pour gauche
        self.no_detection_counter = 0
        self.current_yaw = 0.0
        self.target_yaw = None
        self.robot_position_x = 0.0
        self.robot_position_y = 0.0
        self.start_position_x = 0.0
        self.start_position_y = 0.0
        self.move_forward_distance = 0.5

        # PID pour rotation
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.01
        self.prev_error = 0.0
        self.integral = 0.0

        # Timer pour rotation (50 Hz)
        self.rotation_timer = self.create_timer(0.02, self.rotation_callback)
        self.rotation_timer.cancel()

    def imu_callback(self, msg):
        """Mise à jour de l'angle yaw à partir de l'IMU"""
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def position_callback(self, msg):
        """Mise à jour de la position du robot"""
        self.robot_position_x = msg.pose.pose.position.x
        self.robot_position_y = msg.pose.pose.position.y

    def image_callback(self, msg):
        """Traitement des images pour détecter les rangées"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            center_offset = self.detect_corn_rows(cv_image)  # À implémenter selon votre logique

            if center_offset is not None:
                self.no_detection_counter = 0
                if self.status in ["rotating", "move_forward_after_turn"]:
                    self.get_logger().info("Rangées détectées, passage à navigation")
                    self.status = "navigate"
                self.navigate_to_center(center_offset)
            else:
                self.no_detection_counter += 1
                if self.status == "move_forward_after_turn":
                    cmd = Twist()
                    cmd.linear.x = self.linear_speed
                    self.cmd_pub.publish(cmd)
                    distance_moved = math.sqrt(
                        (self.robot_position_x - self.start_position_x)**2 +
                        (self.robot_position_y - self.start_position_y)**2
                    )
                    if distance_moved >= self.move_forward_distance:
                        self.status = "navigate"
                elif self.no_detection_counter > 10 and self.status == "navigate":
                    self.start_turn()
                    self.status = "rotating"
                    self.rotation_timer.reset()
                else:
                    self.stop_robot()

        except Exception as e:
            self.get_logger().error(f"Erreur traitement image: {e}")

    def start_turn(self):
        """Définir l'angle cible pour un demi-tour"""
        if self.current_yaw is not None:
            self.target_yaw = self.normalize_angle(self.current_yaw + math.pi)
            self.get_logger().info(f"Début rotation : yaw actuel={self.current_yaw}, cible={self.target_yaw}")
            self.prev_error = 0.0
            self.integral = 0.0

    def rotation_callback(self):
        """Contrôle de la rotation avec PID"""
        if self.status != "rotating":
            self.rotation_timer.cancel()
            return
        cmd = Twist()
        if self.current_yaw is not None and self.target_yaw is not None:
            yaw_diff = self.normalize_angle(self.current_yaw - self.target_yaw)
            # PID
            p_term = self.kp * yaw_diff
            self.integral += yaw_diff * 0.02
            i_term = self.ki * self.integral
            d_term = self.kd * (yaw_diff - self.prev_error) / 0.02
            angular_speed = -(p_term + i_term + d_term) * self.rotation
            angular_speed = max(-1.0, min(1.0, angular_speed))  # Limite à ±1.0 rad/s
            cmd.angular.z = angular_speed
            self.cmd_pub.publish(cmd)
            self.prev_error = yaw_diff
            if abs(yaw_diff) < 0.05:
                self.status = "move_forward_after_turn"
                self.start_position_x = self.robot_position_x
                self.start_position_y = self.robot_position_y
                self.rotation *= -1

    def navigate_to_center(self, offset):
        """Navigation vers le centre des rangées"""
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = -offset * 0.1  # Ajustement proportionnel
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Arrêt du robot"""
        self.cmd_pub.publish(Twist())

    def detect_corn_rows(self, image):
        """À implémenter : détection des rangées dans l'image"""
        # Exemple : retourner un décalage (offset) ou None si non détecté
        return None  # Remplacez par votre logique

    def normalize_angle(self, angle):
        """Normalisation de l'angle entre -π et π"""
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

if __name__ == "__main__":
    main()