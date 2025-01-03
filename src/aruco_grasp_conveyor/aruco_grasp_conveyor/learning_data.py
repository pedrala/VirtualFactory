import os
import time
import random
import cv2
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, String

# Constants
r1 = 130  # Length between J0 and J1
r2 = 124  # Length between J1 and J2
r3 = 126  # Length between J2 and J3
th1_offset = -math.atan2(0.024, 0.128)
th2_offset = -0.5 * math.pi - th1_offset

# Helper functions
def solv2(r1, r2, r3):
    d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)
    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)
    return s1, s2

def solv_robot_arm2(x, y, z, r1, r2, r3):
    Rt = math.sqrt(x**2 + y**2 + z**2)
    Rxy = math.sqrt(x**2 + y**2)
    St = math.asin(z / Rt)
    Sxy = math.atan2(y, x)
    s1, s2 = solv2(r1, r2, Rt)
    sr1 = math.pi / 2 - (s1 + St)
    sr2 = s1 + s2
    sr3 = math.pi - (sr1 + sr2)
    return sr1, sr2, sr3, Sxy

# Node class
class Learning(Node): 
    def __init__(self): 
        super().__init__('turtlebot3_manipulation_test')
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.command_sub = self.create_subscription(String, "/learning_test", self.command_callback, 10)

        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.x, self.y, self.z = 100, 0, 200
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        
        self.is_learning = False  # Flag to control learning process
        
    def command_callback(self, msg):
        """Callback to handle commands."""
        print(f"Received message: {msg.data}")  # 수신된 메시지 출력
        if msg.data == "Learning Start" and not self.is_learning:
            self.is_learning = True
            self.get_logger().info("Learning started!")

        elif msg.data == "Learning Stop" and self.is_learning:
            self.is_learning = False
            self.get_logger().info("Learning stopped by command!")

    def update_position(self):
        sr1, sr2, sr3, Sxy = solv_robot_arm2(self.x, self.y, self.z, r1, r2, r3)
        self.joint_angles[0] = Sxy
        self.joint_angles[1] = sr1 + th1_offset
        self.joint_angles[2] = sr2 + th2_offset
        self.joint_angles[3] = sr3
        self.update_trajectory()

    def update_trajectory(self):
        point = JointTrajectoryPoint()
        point.positions = self.joint_angles
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 3
        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)
        time.sleep(0.5)

def capture_images(node, camera_index=0, duration=30):
    os.makedirs("box_image", exist_ok=True)
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Unable to access the camera")
        return

    start_time = time.time()

    while rclpy.ok():
        # Allow ROS to process callbacks
        rclpy.spin_once(node, timeout_sec=0.1)

        if node.is_learning:
            if time.time() - start_time > duration:
                node.is_learning = False
                node.get_logger().info(f"Learning stopped after {duration} seconds!")
                continue

            node.x = random.randint(100, 150)
            node.y = random.randint(-100, 100)
            node.z = random.randint(100, 150)

            node.update_position()

            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                continue

            file_name = f"box_image/box_image_{node.x}_{node.y}_{node.z}.jpg"
            cv2.imwrite(file_name, frame)
            print(f"Captured image: {file_name}")
        else:
            print("Waiting for 'Learning Start' command...")
            time.sleep(1)

    cap.release()


def main():
    rclpy.init()
    node = Learning()
    try:
        capture_images(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
