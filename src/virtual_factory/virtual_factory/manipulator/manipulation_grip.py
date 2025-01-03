import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from std_msgs.msg import Header, String
from ultralytics import YOLO
import os
import cv2
import math
import time
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 상수 정의
r1 = 130
r2 = 124
r3 = 126
th1_offset = -math.atan2(0.024, 0.128)  # 첫 번째 관절의 오프셋 각도
th2_offset = -0.5 * math.pi - th1_offset  # 두 번째 관절의 오프셋 각도

def solv2(r1, r2, r3):
    """링크 간의 각도를 계산하는 함수."""
    d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)
    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)
    return s1, s2

def solv_robot_arm2(x, y, z_fixed, r1, r2, r3):
    """로봇 팔이 주어진 (x, y, z_fixed) 위치에 도달하기 위한 관절 각도를 계산하는 함수."""
    Rt = math.sqrt(x**2 + y**2 + z_fixed**2)
    if Rt > (r1 + r2 + r3):  # 목표 위치가 로봇의 도달 범위를 벗어났는지 확인
        raise ValueError("목표 위치가 로봇의 도달 범위를 벗어났습니다.")

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
        
        # QoS 프로파일 생성
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            depth=10
        )
        
        self.command_sub = self.create_subscription(
            String, 
            "/learning_test", 
            self.command_callback, 
            qos_profile=qos_profile 
        )
        
        self.subscription = self.create_subscription(
            String,
            'status_topic',
            self.status_callback,
            qos_profile=qos_profile
        )
                
        # 로봇 팔의 조인트 트래젝토리를 퍼블리시하는 퍼블리셔 생성
        self.joint_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            qos_profile=qos_profile 
        )
        
        # 그리퍼 액션 클라이언트 생성
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')       

        # 그리퍼 이미지를 퍼블리시하는 퍼블리셔 생성
        self.hand_image_pub = self.create_publisher(Image, '/hand_image',  qos_profile=qos_profile )  
        
        self.is_learning = False            
        
        # OpenCV 이미지를 ROS 2 Image 메시지로 변환하기 위한 CvBridge 생성
        self.bridge = CvBridge()
        
        # 잡목록에서 사용자가 잡을 선택했을 시 구독받음
        self.target_counts_sub = self.create_subscription(
            String,  
            '/target_counts', 
            self.target_counts_callback, 
            qos_profile
        )
        self.target_counts = {"RED": 0, "BLUE": 0}  # 기본 타겟 카운트 설정

        # 트래젝토리 메시지 설정
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # YOLO 모델 로드 (YOLOv8 모델 경로를 본인의 환경에 맞게 변경하세요)
        self.yolo_model = YOLO("/home/viator/ws/aruco/virtual_factory/src/virtual_factory/virtual_factory/manipulator/box.pt")
        self.get_logger().info("로봇 매니퓰레이션 서버가 준비되었습니다.")

        # 초기 위치로 이동
        self.move_to_initial_position()
        
    def command_callback(self, msg):
        """Handle received commands."""
        self.get_logger().warning(f"Received message: {msg.data}")
        if msg.data == "Learning Start":
            self.is_learning = True
        elif msg.data == "Learning Stop":
            self.is_learning = False           
    
    
    def status_callback(self, msg):
        """상태 메시지를 처리하는 콜백 함수."""
        status = msg.data
        self.get_logger().info(f'Received status: {status}')
    
        # 상태에 따라 동작 수행
        if status in ('box_initial_pose', 'put_box', 'put_initial_goal'):
            self.capture_requested = True
            self.timer = self.create_timer(0.1, self.capture_and_publish)
        elif status in ('pick_box', 'picking_basket', 'error'):
            self.capture_requested = False
            self.timer = self.create_timer(0.1, self.capture_and_publish)
        else:
            self.get_logger().info(f'Unhandled status received: {status}')
    
    def capture_and_publish(self):
        """캡처 신호를 수신했을 때 이미지를 캡처하고 퍼블리시"""
        if self.capture_requested:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture image from webcam.")
                return            

            try:
                # OpenCV 이미지를 ROS 메시지로 변환
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                
                # 핸드 이미지 퍼블리시
                self.hand_image_pub.publish(image_message)
                
                if self.is_learning:
                    self.save_image(frame)
                
            except Exception as e:
                self.get_logger().error(f"Failed to process image: {e}")

    def save_image(self, frame):
        """현재 프레임을 파일로 저장."""
        os.makedirs("learning_data", exist_ok=True)
        timestamp = int(time.time())
        file_name = f"learning_data/captured_{timestamp}.jpg"
        cv2.imwrite(file_name, frame)
        self.get_logger().info(f"Saved image: {file_name}")


    def target_counts_callback(self, msg):
        """
        서브스크라이브한 토픽으로부터 타겟 카운트를 업데이트하는 콜백 함수.
        """
        try:
            self.target_counts = json.loads(msg.data)  # JSON 문자열 파싱
            self.get_logger().info(f"업데이트된 타겟 카운트: {self.target_counts}")

            # 타겟 카운트 업데이트 후 객체 감지 및 집기를 수행
            self.detect_and_grasp_with_count()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"타겟 카운트 파싱 실패: {e}")

    def detect_and_grasp_with_count(self):
        """
        YOLO를 사용하여 객체를 감지하고 업데이트된 타겟 카운트에 따라 집는 함수.
        """
        target_counts = self.target_counts  # 최신 타겟 카운트 사용
        self.move_to_initial_position()

        # 웹캠 열기
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            error_message = "웹캠을 열 수 없습니다."
            self.get_logger().error(error_message)
            
            # 카메라 에러 퍼블리시
            error_msg = String()
            error_msg.data = error_message
            return

        ret, frame = cap.read()
        if not ret:
            error_message = "웹캠에서 프레임을 읽는 데 실패했습니다."
            self.get_logger().error(error_message)
            
            # 카메라 에러 퍼블리시
            error_msg = String()
            error_msg.data = error_message
            cap.release()
            return

        # YOLOv8 객체 감지
        results = self.yolo_model(frame)

        if len(results[0].boxes) == 0:
            self.get_logger().info("감지된 객체가 없습니다.")
            cap.release()
            return

        # 영역 정의 (이미지 좌표 기준)
        regions = {
            1: (0, 0, 213, 240),
            2: (213, 0, 426, 240),
            3: (426, 0, 640, 240),
            4: (0, 240, 213, 480),
            5: (213, 240, 426, 480),
            6: (426, 240, 640, 480)
        }

        # 영역을 로봇 좌표로 매핑
        region_to_coords = {
            1: (240, 55, 50),
            3: (240, -55, 50),
            4: (175, 60, 45),
            6: (175, -60, 45)
        }

        # YOLO 클래스 인덱스에서 이름으로 매핑
        class_map = {
            0: "RED",   # YOLO 클래스 0을 "RED"로 매핑
            1: "BLUE"   # YOLO 클래스 1을 "BLUE"로 매핑
        }

        detected_objects = []  # 감지된 객체를 저장할 리스트
        # 모든 감지된 객체 처리
        for box in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # 바운딩 박스 좌표
            detected_class_index = int(box.cls[0])  # 클래스 인덱스 가져오기
            detected_class = class_map.get(detected_class_index, None)  # 클래스 이름으로 매핑

            if detected_class is None:
                self.get_logger().error(f"알 수 없는 클래스 인덱스: {detected_class_index}. 객체를 건너뜁니다.")
                continue

            # 객체가 속한 영역 확인
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

        # 클래스별 감지된 객체 수 세기
        detected_counts = {"RED": 0, "BLUE": 0}
        for obj in detected_objects:
            obj_class = obj["class"]
            detected_counts[obj_class] += 1

        # 타겟 카운트와 감지된 카운트의 일치 여부 확인
        for obj_class, target_count in target_counts.items():
            if detected_counts.get(obj_class, 0) != target_count:
                self.get_logger().error(
                    f"{obj_class}의 불일치: 타겟 카운트는 {target_count}, 하지만 감지된 수는 {detected_counts.get(obj_class, 0)}입니다."
                )
                self.get_logger().error("객체 개수 불일치로 인해 작업을 중지합니다.")
                return  # 카운트 불일치 시 중지

        self.get_logger().info("모든 객체 개수가 타겟 개수와 일치합니다. 집기를 진행합니다.")

        # 클래스별 객체 수 세기
        counts = {"RED": 0, "BLUE": 0}

        for obj in detected_objects:
            obj_class = obj["class"]
            
            if obj_class not in target_counts:
                self.get_logger().warning(f"{obj_class} 클래스는 타겟 카운트에 없습니다. 건너뜁니다.")
                continue

            if counts[obj_class] < target_counts.get(obj_class, 0):
                
                counts[obj_class] += 1
                
                coords = obj["coords"]
                try:
                    # 그리퍼 열기
                    self.control_gripper(0.025)
                    self.get_logger().info("그리퍼를 열었습니다.")

                    # 감지된 객체로 이동
                    self.x, self.y, self.z = coords
                    self.update_position()
                    self.get_logger().info(f"{coords} 위치의 {obj_class} 객체로 이동했습니다.")
                    time.sleep(4)

                    # 그리퍼 닫아 집기
                    self.control_gripper(-0.005)
                    self.get_logger().info(f"그리퍼를 닫았습니다. {obj_class} 객체를 집었습니다.")
                    time.sleep(2)

                    # 초기 위치로 이동
                    self.move_to_initial_position()
                    self.get_logger().info("초기 위치로 돌아갔습니다.")
                    time.sleep(2)

                    # 컨베이어로 이동하고 객체 릴리스
                    self.move_to_conveyor__position()
                    self.get_logger().info(f"{obj_class} 객체와 함께 컨베이어 위치로 이동했습니다.")
                    time.sleep(3)

                    # 그리퍼 열어 객체 놓기
                    self.control_gripper(0.025)
                    self.get_logger().info(f"{obj_class} 객체를 컨베이어에 놓았습니다.")
                    time.sleep(2)

                    # 초기 위치로 돌아오기
                    self.move_to_initial_position()
                    self.get_logger().info("초기 위치로 돌아갔습니다.")
                    time.sleep(2)

                except ValueError as e:
                    self.get_logger().error(f"{coords} 위치의 {obj_class} 객체 집는 중 오류 발생: {e}")
                    continue

        cap.release()

    def move_to_initial_position(self):
        """로봇을 초기 위치 (100, 0, 200)으로 이동시키는 함수."""
        self.x, self.y, self.z = 100, 0, 200
        try:
            self.update_position()  # 위치 업데이트 및 트래젝토리 퍼블리시
            self.get_logger().info("초기 위치로 이동했습니다.")
        except ValueError as e:
            self.get_logger().error(f"초기 위치로 이동 실패: {str(e)}")
    
    def move_to_conveyor__position(self):
        """로봇을 컨베이어 위치 (60, -210, 100)으로 이동시키는 함수."""
        self.x, self.y, self.z = 60, -210, 100
        try:
            self.update_position()  # 위치 업데이트 및 트래젝토리 퍼블리시
            self.get_logger().info("컨베이어 위치로 이동했습니다.")
        except ValueError as e:
            self.get_logger().error(f"컨베이어 위치로 이동 실패: {str(e)}")

    def update_position(self):
        """관절 각도를 계산하고 트래젝토리 메시지를 업데이트하는 함수."""
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        sr1, sr2, sr3, Sxy = solv_robot_arm2(self.x, self.y, self.z, r1, r2, r3)
        self.joint_angles[0] = Sxy
        self.joint_angles[1] = sr1 + th1_offset
        self.joint_angles[2] = sr2 + th2_offset
        self.joint_angles[3] = sr3
        self.update_trajectory()

    def update_trajectory(self):
        """트래젝토리 포인트을 업데이트하고 퍼블리시하는 함수."""
        point = JointTrajectoryPoint()
        point.positions = self.joint_angles
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 3  # 목표 도달 시간 설정
        self.trajectory_msg.points = [point]

        self.joint_pub.publish(self.trajectory_msg)  # 트래젝토리 퍼블리시
        self.get_logger().info(f"관절 각도 {self.joint_angles}으로 트래젝토리를 퍼블리시했습니다.")
        time.sleep(0.5)

    def move_to_position(self, joint_angles):
        """원하는 위치로 로봇 팔을 이동시키기 위한 조인트 트래젝토리를 퍼블리시하는 함수."""
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 2  # 시간 조정 가능
        traj.points.append(point)

        self.joint_pub.publish(traj)  # 트래젝토리 퍼블리시
        self.get_logger().info(f"관절 각도 {joint_angles}으로 이동 중입니다.")

    def control_gripper(self, position):
        """그리퍼를 제어하기 위한 명령을 비동기적으로 보내는 함수."""
        # 그리퍼를 위한 목표 생성
        goal = GripperCommand.Goal()
        goal.command.position = position

        # 그리퍼 명령을 보내기 전에 이미지 캡처 및 퍼블리시
        self.capture_and_publish_image()

        # 목표를 비동기적으로 보내기
        future = self.gripper_action_client.send_goal_async(goal)
        future.add_done_callback(self._handle_gripper_goal_response)
    
    def _handle_gripper_goal_response(self, future):
        """그리퍼 목표에 대한 응답을 처리하는 함수."""
        goal_handle = future.result()

        if not goal_handle:
            self.get_logger().error("그리퍼 명령 전송 실패.")
            return

        if not goal_handle.accepted:
            self.get_logger().error("그리퍼 명령이 수락되지 않았습니다.")
            return

        self.get_logger().info("그리퍼 명령이 수락되었습니다. 결과를 기다립니다.")

        # 결과를 비동기적으로 기다림
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_gripper_result)

    def _handle_gripper_result(self, future):
        """그리퍼 액션의 결과를 처리하는 함수."""
        result = future.result()

        if result.result:
            self.get_logger().info("그리퍼가 원하는 위치로 성공적으로 이동했습니다.")
        else:
            self.get_logger().error("그리퍼 명령 실행에 실패했습니다.")

    def capture_and_publish_image(self):
        """웹캠에서 이미지를 캡처하고 /gripper_image 토픽으로 퍼블리시하는 함수."""
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("이미지 캡처를 위한 카메라를 열 수 없습니다.")
            return

        ret, frame = cap.read()
        if not ret:
            self.get_logger().error("카메라에서 프레임을 캡처하는 데 실패했습니다.")
            cap.release()
            return

        cap.release()

        # 이미지를 ROS Image 메시지로 변환
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # 이미지 퍼블리시
        self.hand_image_pub.publish(image_msg)
        self.get_logger().info("/gripper_image 토픽으로 이미지를 퍼블리시했습니다.") 

def main(args=None):
    rclpy.init(args=args)
    node = RobotManipulationServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 topic pub /target_counts std_msgs/msg/String '{"data": "{\"RED\": 2, \"BLUE\": 1}"}' --once