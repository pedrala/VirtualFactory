#!/usr/bin/env python3
import rclpy  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2 노드 생성을 관리
from sensor_msgs.msg import Image  # 센서 메시지 타입 - 이미지
from std_msgs.msg import Int32, String  # ArUco Zone 메시지 타입
from cv_bridge import CvBridge, CvBridgeError  # ROS 2와 OpenCV 이미지 변환 라이브러리
import cv2  # Python OpenCV 라이브러리
import numpy as np  # 수치 계산을 위한 라이브러리
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
from rclpy.executors import MultiThreadedExecutor  # Executor 추가
from virtual_factory_if.msg import RelativePosition  # 커스텀 메시지 임포트

# ArUco 마커 딕셔너리 선택
aruco_dict_type = cv2.aruco.DICT_5X5_100  # 5x5 ArUco 마커 딕셔너리
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
aruco_params = cv2.aruco.DetectorParameters()

# 카메라 파라미터 로드 (camera_matrix, dist_coeffs)
camera_matrix = np.load('/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/npy/camera_matrix.npy')  # 카메라 내부 파라미터
dist_coeffs = np.load('/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/npy/distortion_coefficients.npy')  # 왜곡 계수

# 마커 크기 (미터 단위)
marker_length = 0.1  # 10cm

# 결과 이미지를 저장할 폴더 설정
image_folder = "/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/aruco_images"
output_folder = "/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/result_marker"
os.makedirs(output_folder, exist_ok=True)

# 기준 마커 ID 설정
base_marker_id = 2  # 기준 마커의 ID (예: ID 2이 기준)

# 기준 마커의 상대 좌표 (0, 0, 0)
base_marker_position = np.array([0.0, 0.0, 0.0])

class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        
        # QoS 프로파일 생성
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            depth=10
        )

        # CvBridge 초기화
        self.bridge = CvBridge()
        
        # USB 카메라 초기화
        self.cap = cv2.VideoCapture(2)  # 2번 디바이스 (USB 카메라)
        if not self.cap.isOpened():
            self.get_logger().error("USB 카메라를 열 수 없습니다.")
            rclpy.shutdown()
            return

        # 타겟 마커 ID 초기화
        self.target_marker_id = None  # 초기에는 None으로 설정

        # 타겟 마커 ID를 받아오는 구독자 설정
        self.target_marker_sub = self.create_subscription(
            Int32,
            'target_marker_id',
            self.target_marker_id_callback,
            qos_profile=qos_profile
        )
        
        # 상대 위치를 위한 퍼블리셔 설정 (커스텀 메시지)
        self.relative_position_pub = self.create_publisher(
            RelativePosition, 
            'marker_relative_position',
            qos_profile=qos_profile 
        )       
        
        self.aruco_detection_pub = self.create_publisher(
            Image, 
            'aruco_detected_image',
            qos_profile=qos_profile 
        )       

        self.get_logger().info("Aruco Detection Node Started")

        self.zones = {  # 구역을 나타내는 값들
            'initial_pos': 0,   # 최초 위치
            'picking_pos': 1,   # 박스 위치 and 바스켓 위치
            'loading_pos': 2    # 하역장 위치
        }
        self.zone = 0  # 기본 구역 값

        # 타이머 설정 (예: 10Hz)
        timer_period = 0.1  # 초 단위
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("ArucoDetection Node Initialized")

    
    def target_marker_id_callback(self, msg):
        """GUI 서버로부터 타겟 마커 ID를 받아 업데이트합니다."""
        self.target_marker_id = msg.data
        self.get_logger().info(f"타겟 마커 ID가 {self.target_marker_id}(으)로 업데이트 되었습니다.")

        
    def timer_callback(self):
        """
        주기적으로 호출되는 타이머 콜백 함수.
        USB 카메라로부터 이미지를 캡처하여 ArUco 마커를 탐지하고 처리합니다.
        """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("카메라에서 프레임을 읽는 데 실패했습니다.")
            return
        
        self.aruco_detect_markers(frame)

    # 상대 위치 계산 함수
    def calculate_relative_position(self, base_rvec, base_tvec, target_rvec, target_tvec):
        """
        기준 마커에서 타겟 마커의 상대 위치 및 방향 계산 (x, z축만을 고려).

        Parameters:
            base_rvec: 기준 마커의 회전 벡터 (3x1 numpy 배열)
            base_tvec: 기준 마커의 이동 벡터 (3x1 numpy 배열)
            target_rvec: 타겟 마커의 회전 벡터 (3x1 numpy 배열)
            target_tvec: 타겟 마커의 이동 벡터 (3x1 numpy 배열)

        Returns:
            relative_tvec_xz: 기준 좌표계에서 타겟 마커의 상대 위치 (x, z 좌표만 포함)
            distance: 기준 마커에서 타겟 마커까지의 거리
            angle_rad: 상대 방향 벡터의 각도 (라디안 단위, 기준: x축)
            quaternion: 회전을 나타내는 쿼터니언 (x, y, z, w)
            rx, ry, rz: 오일러 각도 (롤, 피치, 요) 라디안 단위
        """
        # 기준 마커의 회전 행렬 계산
        R_base, _ = cv2.Rodrigues(base_rvec)  # 기준 마커의 회전 행렬
        R_base_inv = np.linalg.inv(R_base)    # 회전 행렬의 역행렬

        # 타겟 마커의 이동 벡터를 기준 좌표계로 변환
        tvec_relative = R_base_inv @ (target_tvec.T - base_tvec.T)

        # z, x 성분만 추출
        tvec_relative_xz = tvec_relative[[0, 2]]  # z, x 좌표 (상대 이동)
        distance = np.linalg.norm(tvec_relative_xz)  # 거리 계산
        if distance == 0:
            angle_rad = 0.0
            direction_vector = np.array([1.0, 0.0])
        else:
            direction_vector = tvec_relative_xz / distance  # 방향 벡터 (단위 벡터)

        # z, x를 기반으로 각도 계산 (atan2 사용)
        angle_rad = np.arctan2(tvec_relative_xz[1], tvec_relative_xz[0])  # 라디안 단위

        # scipy를 사용하여 쿼터니언으로 변환
        r = R.from_euler('z', angle_rad)  # 'z'축 회전만 고려
        quaternion = r.as_quat()  # [x, y, z, w]

        # 오일러 각도로 변환 (라디안 단위)
        rx, ry, rz = r.as_euler('xyz', degrees=False)

        return tvec_relative_xz.flatten(), distance, angle_rad, quaternion, rx, ry, rz

    # ArUco 마커 탐지 및 처리 함수
    def aruco_detect_markers(self, cv_image):
        self.get_logger().info('아루코 마커 탐지 시작...')

        # ArUco 마커 탐지
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_params)

        if ids is not None:
            self.get_logger().info(f"[INFO] Detected markers: {ids.flatten()}")
            # 마커의 자세 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            base_rvec, base_tvec = None, None
            target_rvec, target_tvec = None, None

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == base_marker_id:  # 기준 마커
                    base_rvec, base_tvec = rvecs[i], tvecs[i]
                    self.get_logger().info(f"기준 마커(ID {marker_id}) 발견.")
                    
                elif marker_id == self.target_marker_id:  # GUI 서버에서 지정한 타겟 마커
                    target_rvec, target_tvec = rvecs[i], tvecs[i]
                    self.get_logger().info(f"타겟 마커(ID {marker_id}) 발견.")

                # 기준과 타겟 마커가 모두 발견되면 상대 위치 계산
                if base_rvec is not None and base_tvec is not None and target_rvec is not None and target_tvec is not None:
                    relative_tvec, distance, angle_rad, quaternion, rx, ry, rz = self.calculate_relative_position(
                        base_rvec, base_tvec, target_rvec, target_tvec
                    )           
                    self.get_logger().info('상대 위치 계산 완료.')

                    # 메시지 생성
                    msg = RelativePosition()
                    msg.marker_id = self.target_marker_id
                    msg.x = relative_tvec[0]
                    msg.y = 0.0  # y값은 사용하지 않음
                    msg.z = relative_tvec[1]
                    msg.distance = distance        

                    # 각도 출력 (디버깅 용도)
                    print(f"Type of angle_rad: {type(angle_rad)}")
                    print(f"angle_rad: {angle_rad:.6f}")

                    # angle_rad가 numpy.ndarray일 경우 첫 번째 값만 추출하여 할당
                    if isinstance(angle_rad, np.ndarray):
                        msg.angle = angle_rad[0]  # 첫 번째 값만 사용
                    else:
                        msg.angle = angle_rad  # 이미 float이면 그대로 사용

                    # Quaternion 객체 생성 및 할당            
                    msg.quaternion = Quaternion()
                    msg.quaternion.x = quaternion[0]
                    msg.quaternion.y = quaternion[1]
                    msg.quaternion.z = quaternion[2]
                    msg.quaternion.w = quaternion[3]

                    # 쿼터니언에서 오일러 각도 (롤, 피치, 요) 추출
                    msg.rx = rx
                    msg.ry = ry
                    msg.rz = rz
                    roll, pitch, yaw = rx, ry, rz

                    # 로그에 오일러 각도 추가
                    self.get_logger().info(f"Roll (rx): {roll:.6f} radians")
                    self.get_logger().info(f"Pitch (ry): {pitch:.6f} radians")
                    self.get_logger().info(f"Yaw (rz): {yaw:.6f} radians")

                    self.get_logger().info(f"marker_id: {msg.marker_id}")
                    self.get_logger().info(f"relative position: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
                    self.get_logger().info(f"distance: {msg.distance:.3f} meters")
                    self.get_logger().info(f"angle (radians): {msg.angle:.6f}")
                    self.get_logger().info(f"quaternion: x={msg.quaternion.x}, y={msg.quaternion.y}, z={msg.quaternion.z}, w={msg.quaternion.w}")                    

                    # 퍼블리시할 메시지 생성 (RelativePosition 타입으로 퍼블리시)
                    self.relative_position_pub.publish(data=msg)                    
                    
                    # 이미지에 ArUco 마커와 축 그리기
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                    cv2.aruco.drawAxis(cv_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length)  # 축 그리기 (한 번만 그리기)

                    # 처리된 이미지를 화면에 표시
                    # cv2.imshow("ArUco Markers", cv_image)
                    # cv2.waitKey(1)  # OpenCV 창을 업데이트
                    
                    # 처리된 이미지를 GUI server에서 받도록 퍼블리쉬
                    self.aruco_detection_pub.publish(cv_image)      
                    
                    # 상대 위치 하나 처리 후 루프 종료
                    break
                
               else:
                self.get_logger().warn(f"기준 마커(ID {base_marker_id}) 또는 타겟 마커(ID {self.target_marker_id})를 찾을 수 없음.")

        else:
            self.get_logger().info("아루코 마커를 찾을 수 없습니다.")

    def main(args=None):
        """
        메인 함수: 노드 초기화 및 실행
        """
        rclpy.init(args=args)  # ROS 2 초기화
        aruco_detection = ArucoDetection()  # ArUco 탐지 노드 생성
        # spin()을 사용하여 이벤트 루프를 계속 실행
        try:
            rclpy.spin(aruco_detection)
        except KeyboardInterrupt:
            pass
        finally:
            aruco_detection.get_logger().info("Shutting down the node.")  # 종료 직전 로깅
            if aruco_detection.cap.isOpened():
                aruco_detection.cap.release()  # 카메라 자원 해제
            cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기
            aruco_detection.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()
