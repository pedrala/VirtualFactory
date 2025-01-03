import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import time



class HandiCamPublisher(Node):
    def __init__(self):
        super().__init__('handi_cam_publisher')
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Image, 'hand_image', 10)
        
        # 구독자 생성
        self.subscription = self.create_subscription(
            String,
            'status_topic',
            self.status_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            "learning_test",
            self.command_callback,
            10
        )
        
        self.is_learning = False

        # CvBridge 객체 생성
        self.bridge = CvBridge()
        
        # OpenCV 웹캠 초기화
        self.cap = cv2.VideoCapture("/dev/video2")  # 0번 카메라 (기본 웹캠)
        if not self.cap.isOpened():
            self.get_logger().error("Unable to open webcam.")
            raise RuntimeError("Webcam could not be opened.")
        
        self.capture_requested = False
        self.get_logger().info("HandiCamPublisher node initialized.")

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
                
                # 이미지 퍼블리시
                self.publisher_.publish(image_message)
                # self.get_logger().info("Published hand image.")
                
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

    def destroy_node(self):
        """노드 종료 시 리소스 정리"""
        self.cap.release()  # 웹캠 리소스 해제
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = HandiCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
