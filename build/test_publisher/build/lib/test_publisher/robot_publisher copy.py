#!/usr/bin/env python3
"""
Script to move Robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
import time
import cv2
from cv_bridge import CvBridge


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        # Create a publisher which can "talk" to Robot and tell it to move
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )
        self.camera_info = None

        # Subscriber pour la caméra
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            CameraInfo,
            "/zed2_left_raw/camera_info",  # ← le topic ROS
            self.image_callback,
            10
        )

        self.run()
        

    def image_callback(self, msg):
        """Callback pour afficher les données reçues de la caméra"""
        try:
            self.camera_info = msg
            # Afficher les informations du message CameraInfo
            self.get_logger().info(f"\n\nCamera Info Received: {msg}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la réception des données de la caméra : {e}")


    def run(self):
        # Create a Twist message and add linear x and angular z values
        move_cmd = Twist()

        ######## Move Straight ########
        print("Moving Straight")
        move_cmd.linear.x = 0.5  # move in x axis at 0.5 m/s
        move_cmd.angular.z = 0.0
        self.sub

        now = time.time()
        # For the next 4 seconds publish cmd_vel move commands
        while time.time() - now < 4:
            self.pub.publish(move_cmd)  # publish to robot

        ######## Stop ########
        print("Stopping")
        # Assigning both to 0.0 stops the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 5 seconds publish cmd_vel move commands
        while time.time() - now < 5:
            self.pub.publish(move_cmd)

        ######## Rotating Counterclockwise ########
        print("Rotating")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7  # rotate at 0.7 rad/s

        now = time.time()
        # For the next 15 seconds publish cmd_vel move commands
        while time.time() - now < 15:
            self.pub.publish(move_cmd)

        ######## Stop ########
        print("Stopping")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 3 seconds publish cmd_vel move commands
        while time.time() - now < 3:
            self.pub.publish(move_cmd)

        print("Exit")


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()

    # Spin the node so the callback function is called
    rclpy.spin(move_robot)
    # Destroy the node explicitly
    move_robot.destroy_node()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
