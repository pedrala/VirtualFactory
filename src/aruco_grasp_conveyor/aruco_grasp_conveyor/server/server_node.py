import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from manipulator_msg.msg import RelativePosition  # 커스텀 메시지 임포트
from manipulator_msg.srv import GoalLocation  # 커스텀 서비스 임포트
from std_msgs.msg import Int32, Float32, String 
import sys
import os

class ServerNode(Node):

    def __init__(self):
        # 서버 노드를 초기화
        super().__init__('server_node')
        print("Current working directory:", os.getcwd())  # 현재 작업 디렉토리 출력
        print("Python path:", sys.path)  # Python 경로 출력

        # QoS 프로파일 생성 (신뢰성: 신뢰성 있는 통신, 큐의 크기: 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # CvBridge 객체 초기화 (OpenCV 이미지와 ROS 이미지 메시지 변환)
        self.bridge = CvBridge()

        # 카메라 원본 이미지 퍼블리셔 생성
        self.image_pub = self.create_publisher(Image, '/camera_raw', qos_profile)

        # USB 카메라를 사용하여 비디오 캡처 객체 초기화
        self.cap = cv2.VideoCapture(2)  # USB 카메라 장치 2 사용
        if not self.cap.isOpened():
            # 카메라 열기 실패 시 로그 출력
            self.get_logger().error("Cannot open camera")
            self.cap = None
            return

        # 0.1초마다 이미지를 캡처하고 퍼블리시하는 타이머 설정
        self.timer = self.create_timer(0.1, self.publish_image)

        # 상대 위치 데이터를 구독하는 서브스크라이버 생성
        self.relative_position_sub = self.create_subscription(
            RelativePosition,
            'marker_relative_position',
            self.relative_position_callback,
            qos_profile=qos_profile
        )

        # 목표 위치를 요청할 수 있는 클라이언트 생성
        self.goal_position = self.create_client(
            GoalLocation,
            'goal_position',
            qos_profile=qos_profile
        )

    def relative_position_callback(self, msg):
        # 상대 위치, 거리 및 각도 처리
        self.get_logger().info(f"marker_id: {msg.marker_id}")
        self.get_logger().info(f"Received relative position: x={msg.x}, z={msg.z}")
        self.get_logger().info(f"Received distance: {msg.distance}")
        self.get_logger().info(f"Received angle (radians): {msg.angle}")
        self.get_logger().info(f"Received quaternion: {msg.quaternion}")

        # GoalLocation 서비스 요청 객체 생성
        request = GoalLocation.Request()
        request.marker_id = msg.marker_id  # marker_id 추가
        request.x = msg.x
        request.z = msg.z
        # Float32 메시지 객체 생성 후 거리 값 할당
        request.distance = Float32()
        request.distance.data = float(msg.distance)  # 거리 값은 float 타입으로 할당
        request.quaternion = msg.quaternion  # 회전 정보 추가

        # ROS2 서비스 호출
        future = self.goal_position.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        # 서비스 호출 후 응답 처리
        try:
            response = future.result()
            self.get_logger().info(f'Success: {response.success}, Message: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_image(self):
        # 카메라가 없으면 이미지를 퍼블리시하지 않음
        if self.cap is None:
            return

        # 카메라에서 프레임 캡처
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        # OpenCV 프레임을 ROS 이미지 메시지로 변환
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # 변환된 이미지를 퍼블리시
        self.image_pub.publish(ros_image)
        self.get_logger().info("Publishing image")

        # OpenCV를 사용해 캡처된 프레임을 화면에 표시 (주석처리됨)
        # cv2.imshow("Frame", frame)
        # cv2.waitKey(1)  # OpenCV 창을 업데이트

    def __del__(self):
        # 노드 종료 시 비디오 캡처 객체 해제
        if self.cap and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # 서버 노드 객체 생성
    node = ServerNode()

    # 노드가 종료될 때까지 실행
    rclpy.spin(node)

    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # main 함수 실행
    main()
