import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from std_msgs.msg import Header, String
from ultralytics import YOLO
import cv2
import math
import time
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Constants
r1 = 130
r2 = 124
r3 = 126
th1_offset = -math.atan2(0.024, 0.128)
th2_offset = -0.5 * math.pi - th1_offset

# Helper Functions
def solv2(r1, r2, r3):
    """Calculate angles between links."""
    d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)
    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)
    return s1, s2

def solv_robot_arm2(x, y, z_fixed, r1, r2, r3):
    """Calculate joint angles for the robot arm to reach the given (x, y, z_fixed)."""
    Rt = math.sqrt(x**2 + y**2 + z_fixed**2)
    if Rt > (r1 + r2 + r3):  # Check if the target position is reachable
        raise ValueError("Target position is out of the manipulator's reach.")

    St = math.asin(z_fixed / Rt)
    Sxy = math.atan2(y, x)
    s1, s2 = solv2(r1, r2, Rt)
    sr1 = math.pi / 2 - (s1 + St)
    sr2 = s1 + s2
    sr3 = math.pi - (sr1 + sr2)
    return sr1, sr2, sr3, Sxy

class RobotManipulationServer(Node):
    def __init__(self):
        super().__init__('robot_manipulation_server')
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

        # Publisher for camera error
        self.camera_error_pub = self.create_publisher(String, '/hand_eye_error', 10)

	# Publisher for gripper images
        self.image_pub = self.create_publisher(Image, '/gripper_image', 10)

        # CvBridge for converting OpenCV images to ROS 2 Image messages
        self.bridge = CvBridge()
        
        # Subscriber for target counts
        self.target_counts_sub = self.create_subscription(
            String,  # Message type
            '/target_counts',  # Topic name
            self.target_counts_callback,  # Callback function
            10  # QoS depth
        )
        self.target_counts = {"RED": 0, "BLUE": 0}  # Default target counts

        # Trajectory message setup
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # Load YOLO model
        self.yolo_model = YOLO("/home/booding/box.pt")  # Replace with your YOLOv8 model path
        self.get_logger().info("Robot Manipulation Server is ready.")

        self.move_to_initial_position()
    
    def target_counts_callback(self, msg):
        """
        Callback to update target counts from the subscribed topic.
        """
        try:
            self.target_counts = json.loads(msg.data)  # Parse JSON string
            self.get_logger().info(f"Updated target counts: {self.target_counts}")

            # Call detect_and_grasp_with_count after updating target counts
            self.detect_and_grasp_with_count()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse target counts: {e}")

    def detect_and_grasp_with_count(self):
        """
        Detect objects using YOLO and grasp them based on the updated target counts.
        """
        target_counts = self.target_counts  # Use the latest subscribed counts
        self.move_to_initial_position()

        # Open webcam
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            error_message = "Could not open webcam."
            self.get_logger().error(error_message)
            
            # Publish camera error
            error_msg = String()
            error_msg.data = error_message
            self.camera_error_pub.publish(error_msg)
            return

        ret, frame = cap.read()
        if not ret:
            error_message = "Failed to read from webcam."
            self.get_logger().error(error_message)
            
            # Publish camera error
            error_msg = String()
            error_msg.data = error_message
            self.camera_error_pub.publish(error_msg)
            cap.release()
            return

        # YOLOv8 object detection
        results = self.yolo_model(frame)

        if len(results[0].boxes) == 0:
            self.get_logger().info("No objects detected.")
            cap.release()
            return

        # Define regions
        regions = {
            1: (0, 0, 213, 240),
            2: (213, 0, 426, 240),
            3: (426, 0, 640, 240),
            4: (0, 240, 213, 480),
            5: (213, 240, 426, 480),
            6: (426, 240, 640, 480)
        }

        # Mapping from region to robot coordinates
        region_to_coords = {
            1: (240, 55, 50),
            3: (240, -55, 50),
            4: (175, 60, 45),
            6: (175, -60, 45)
        }

        # YOLO class index to name mapping
        class_map = {
            0: "RED",   # Map YOLO class 0 to "RED"
            1: "BLUE"   # Map YOLO class 1 to "BLUE"
        }

        detected_objects = []  # Store detected objects with details
        # Process all detected objects
        for box in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # Bounding box coordinates
            detected_class_index = int(box.cls[0])  # Get class index
            detected_class = class_map.get(detected_class_index, None)  # Map to class name

            if detected_class is None:
                self.get_logger().error(f"Unknown class index: {detected_class_index}. Skipping object.")
                continue

            for region_id, (x1, y1, x2, y2) in regions.items():
                if not (x_max < x1 or x_min > x2 or y_max < y1 or y_min > y2):
                    if region_id in region_to_coords:
                        detected_objects.append({
                            "class": detected_class,
                            "region_id": region_id,
                            "coords": region_to_coords[region_id]
                        })
                    break

        cap.release()

        # Count detected objects by class
        detected_counts = {"RED": 0, "BLUE": 0}
        for obj in detected_objects:
            obj_class = obj["class"]
            detected_counts[obj_class] += 1

        # Validate counts against target counts
        for obj_class, target_count in target_counts.items():
            if detected_counts.get(obj_class, 0) != target_count:
                self.get_logger().error(
                    f"Mismatch for {obj_class}: Target count is {target_count}, but detected {detected_counts.get(obj_class, 0)}."
                )
                self.get_logger().error("Object counts mismatch. Stopping operations.")
                return  # Stop execution if counts mismatch

        self.get_logger().info("All object counts match the target counts. Proceeding with grasping.")

        # Count objects by class
        counts = {"RED": 0, "BLUE": 0}

        for obj in detected_objects:
            obj_class = obj["class"]
            
            if obj_class not in target_counts:
                self.get_logger().warning(f"Class {obj_class} is not in target counts. Skipping.")
                continue

            if counts[obj_class] < target_counts.get(obj_class, 0):
                
                counts[obj_class] += 1
                
                coords = obj["coords"]
                try:
                    # Open gripper before moving
                    self.control_gripper(0.025)
                    self.get_logger().info("Gripper opened.")

                    # Move to the detected object
                    self.x, self.y, self.z = coords
                    self.update_position()
                    self.get_logger().info(f"Moved to {obj_class} object at {coords}.")
                    time.sleep(4)

                    # Close gripper to grasp
                    self.control_gripper(-0.005)
                    self.get_logger().info(f"Gripper closed. {obj_class} object grasped.")
                    time.sleep(2)

                    # Move back to the initial position
                    self.move_to_initial_position()
                    self.get_logger().info("Returned to initial position.")
                    time.sleep(2)

                    # Move to conveyor and release
                    self.move_to_conveyor__position()
                    self.get_logger().info(f"Moved to conveyor position with {obj_class} object.")
                    time.sleep(3)

                    # Open gripper to release the object
                    self.control_gripper(0.025)
                    self.get_logger().info(f"{obj_class} object released on conveyor.")
                    time.sleep(2)

                    # Return to initial position
                    self.move_to_initial_position()
                    self.get_logger().info("Returned to initial position.")
                    time.sleep(2)

                except ValueError as e:
                    self.get_logger().error(f"Error while grasping {obj_class} object at {coords}: {e}")
                    continue


        cap.release()

    def move_to_initial_position(self):
        """Move the robot to the initial position (100, 0, 200)."""
        self.x, self.y, self.z = 100, 0, 200
        try:
            self.update_position()  # Update position and publish trajectory
            self.get_logger().info("Moved to initial position.")
        except ValueError as e:
            self.get_logger().error(f"Failed to move to initial position: {str(e)}")
    
    def move_to_conveyor__position(self):
        """Move the robot to the initial position (100, 0, 200)."""
        self.x, self.y, self.z = 60, -210, 100
        try:
            self.update_position()  # Update position and publish trajectory
            self.get_logger().info("Moved to initial position.")
        except ValueError as e:
            self.get_logger().error(f"Failed to move to initial position: {str(e)}")


    def update_position(self):
        """Calculate joint angles and update trajectory message."""
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        sr1, sr2, sr3, Sxy = solv_robot_arm2(self.x, self.y, self.z, r1, r2, r3)
        self.joint_angles[0] = Sxy
        self.joint_angles[1] = sr1 + th1_offset
        self.joint_angles[2] = sr2 + th2_offset
        self.joint_angles[3] = sr3
        self.update_trajectory()

    def update_trajectory(self):
        """Update trajectory points and publish."""
        point = JointTrajectoryPoint()
        point.positions = self.joint_angles
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 3
        self.trajectory_msg.points = [point]

        self.joint_pub.publish(self.trajectory_msg)
        self.get_logger().info(f"Published trajectory with joint angles: {self.joint_angles}")
        time.sleep(0.5)

    def move_to_position(self, joint_angles):
        """Publish joint trajectory to move the arm to the desired position."""
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 2  # Adjust timing as needed
        traj.points.append(point)

        self.joint_pub.publish(traj)
        self.get_logger().info(f"Moving to joint angles: {joint_angles}")

    def control_gripper(self, position):
        """Send a command to control the gripper asynchronously using add_done_callback."""
        # Create a goal for the gripper
        goal = GripperCommand.Goal()
        goal.command.position = position

        # Capture and publish an image before sending the gripper command
        self.capture_and_publish_image()

        # Send the goal asynchronously
        future = self.gripper_action_client.send_goal_async(goal)
        future.add_done_callback(self._handle_gripper_goal_response)
    
    def _handle_gripper_goal_response(self, future):
        """Handle the response for the gripper goal."""
        goal_handle = future.result()

        if not goal_handle:
            self.get_logger().error("Failed to send gripper command.")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Gripper command was not accepted.")
            return

        self.get_logger().info("Gripper command accepted, waiting for result.")

        # Wait for the result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_gripper_result)

    def _handle_gripper_result(self, future):
        """Handle the result of the gripper action."""
        result = future.result()

        if result.result:
            self.get_logger().info("Gripper successfully moved to the desired position.")
        else:
            self.get_logger().error("Failed to execute gripper command.")

    def capture_and_publish_image(self):
        """Capture an image from the webcam and publish it to the /gripper_image topic."""
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera for image capture.")
            return

        ret, frame = cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera.")
            cap.release()
            return

        cap.release()

        # Convert the image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image
        self.image_pub.publish(image_msg)
        self.get_logger().info("Published image to /gripper_image.") 


def main(args=None):
    rclpy.init(args=args)
    node = RobotManipulationServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 topic pub /target_counts std_msgs/msg/String '{"data": "{\"RED\": 2, \"BLUE\": 1}"}' --once
