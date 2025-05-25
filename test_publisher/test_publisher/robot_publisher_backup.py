import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image, LaserScan, NavSatFix
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class CornRowNavigator(Node):
    def __init__(self):
        super().__init__("corn_row_navigator")

        self.cmd_pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

        self.image_sub = self.create_subscription(
            Image, "/zed2_camera_center/image_raw", self.image_callback, 10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.get_logger().info("Corn Row Navigator Started")

        self.linear_speed = 5.0
        self.angular_gain = 1.0
        self.target_distance = 1.5

        self.no_detection_counter = 0
        self.turn_counter = 0
        self.status = "navigate"
        self.rotation = -1

        self.position_sub = self.create_subscription(
            Odometry, "/odom", self.position_callback, 10
        )
        

        self.goal_position = 1.05, 3.81, 0.35 # Position cible du robot (x, y, z)
        self.robot_position_x = 0.0
        self.robot_position_y = 0.0
        self.robot_current_orientation = None
        self.robot_start_orientation = None
        self.target_yaw = None
        
    def position_callback(self, msg):
        """Callback pour traiter la position du robot"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.robot_position_x = position.x
        self.robot_position_y = position.y
        self.robot_current_orientation = orientation
        if self.status == "navigate":
            self.robot_start_orientation = orientation

        # self.get_logger().info(f"positions : x={position.x},y={position.y}, z={position.z}")
        # self.get_logger().info(f"current_orr: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
        # self.get_logger().info(f"start_orr: x={self.robot_start_orientation.x}, y={self.robot_start_orientation.y}, z={self.robot_start_orientation.z}, w={self.robot_start_orientation.w}")


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            center_offset = self.detect_corn_rows(cv_image)

            if center_offset is not None and self.status == "navigate":
                self.no_detection_counter = 0
                self.navigate_to_center(center_offset)
            else:
                self.no_detection_counter += 1
                if self.no_detection_counter > 10:
                    self.start_turn() if self.status == "navigate" else None
                    self.status = "rotating"
                    self.search_for_rows()
                else:
                    self.stop_robot()

        except Exception as e:
            self.get_logger().error(f"Erreur traitement image: {e}")

    def start_turn(self):
        """Définir l'angle cible pour un demi-tour de 180°"""
        if self.robot_current_orientation is not None:
            self.start_yaw = self.quaternion_to_yaw(self.robot_current_orientation)
            self.target_yaw = self.normalize_angle(self.start_yaw + math.pi)
            self.get_logger().info(f"Début rotation : yaw initial={self.start_yaw}, yaw cible={self.target_yaw}")
            
    def detect_corn_rows(self, image):
        height, width = image.shape[:2]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([62,87,7])
        upper_green = np.array([80,106,32])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        roi_height = int(height * 0.6)
        roi_y_start = height - roi_height
        green_roi = green_mask[roi_y_start:height, :]

        contours_green, _ = cv2.findContours(green_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 🧠 Filtrer les petits contours (herbe)
        min_area = 1000
        valid_contours = [cnt for cnt in contours_green if cv2.contourArea(cnt) > min_area]

        if len(valid_contours) < 2:
            return None

        valid_contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)

        left_row_center = self.get_contour_center(valid_contours[0])
        right_row_center = self.get_contour_center(valid_contours[1])

        if left_row_center[0] > right_row_center[0]:
            left_row_center, right_row_center = right_row_center, left_row_center

        # ✅ Vérifier que les deux sont bien écartées
        if abs(left_row_center[0] - right_row_center[0]) < 150:
            return None

        # ✅ Vérifier qu’il n’y a pas trop de vert au centre
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

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) == 0:
            return

        left_distances = ranges[0:45]
        right_distances = ranges[-45:]
        left_avg = np.mean(left_distances[np.isfinite(left_distances)])
        right_avg = np.mean(right_distances[np.isfinite(right_distances)])

        if np.isfinite(left_avg) and np.isfinite(right_avg):
            distance_diff = left_avg - right_avg
            if abs(distance_diff) > 0.2:
                self.get_logger().info(f"LIDAR correction: L={left_avg:.2f}m, R={right_avg:.2f}m")

    def navigate_to_center(self, center_offset):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        normalized_offset = center_offset / 320.0
        cmd.angular.z = -normalized_offset * self.angular_gain
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        self.cmd_pub.publish(cmd)
        # self.get_logger().info(f"Navigation: offset={center_offset:.1f}px, angular_z={cmd.angular.z:.2f}")

    def turn_right(self):
        cmd = Twist()
        cmd.linear.x = 1.5
        cmd.angular.z = -15.0
        self.cmd_pub.publish(cmd)


        if self.robot_current_orientation is not None and self.target_yaw is not None:
            self.get_logger().info("Tourne à droite")
            yaw_diff = self.normalize_angle(
                self.quaternion_to_yaw(self.robot_current_orientation) 
                - 
                self.target_yaw
                )
            if abs(yaw_diff) < 0.1:
                self.no_detection_counter = 0
                self.turn_counter += 1
                self.status = "navigate"
                self.rotation *= -1  
        

    def turn_left(self):
        cmd = Twist()
        cmd.linear.x = 1.5
        cmd.angular.z = 15.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Tourne à gauche")
        if self.robot_current_orientation is not None and self.target_yaw is not None:
            yaw_diff = self.normalize_angle(
                self.quaternion_to_yaw(self.robot_current_orientation) 
                - 
                self.target_yaw
                )   
            if abs(yaw_diff) < 0.1:
                self.no_detection_counter = 0
                self.turn_counter += 1
                self.status = "navigate"
                self.rotation *= -1

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().warn("Rangées non détectées - Robot arrêté")

    def search_for_rows(self):
        """Tourne lentement pour rechercher les rangées perdues"""
        if self.rotation == -1:
            self.turn_right()
        else:
            self.turn_left()      
    
    def quaternion_to_yaw(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


    def normalize_angle(angle):
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
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


