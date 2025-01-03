#!/usr/bin/env python3
import rclpy # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node # ROS 2 노드 생성을 관리
from sensor_msgs.msg import Image # 센서 메시지 타입 - 이미지
from std_msgs.msg import Int32, String # ArUco Zone 메시지 타입
from cv_bridge import CvBridge, CvBridgeError # ROS 2와 OpenCV 이미지 변환 라이브러리
import cv2 # Python OpenCV 라이브러리
import numpy as np # 수치 계산을 위한 라이브러리
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np
import os
from rclpy.executors import MultiThreadedExecutor  # Executor 추가
from manipulator_msg.msg import RelativePosition  # Import the custom message

# ArUco 마커 딕셔너리 선택
aruco_dict_type = cv2.aruco.DICT_5X5_100  # 5x5 ArUco 마커 딕셔너리
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
aruco_params = cv2.aruco.DetectorParameters()

# 카메라 파라미터 로드 (camera_matrix, dist_coeffs)
camera_matrix = np.load('/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/npy/camera_matrix.npy')  # 카메라 내부 파라미터
dist_coeffs = np.load('/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/npy/distortion_coefficients.npy') # 왜곡 계수

# 마커 크기 (미터 단위)
marker_length = 0.1  # 10cm

# 이미지 폴더
image_folder = "/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/aruco_images" 
output_folder = "/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/result_marker" 
os.makedirs(output_folder, exist_ok=True)

# 기준 마커 ID 설정
base_marker_id = 2  # 기준 마커의 ID (예: ID 2이 기준)

# 기준 마커의 상대 좌표 (0, 0, 0)
base_marker_position = np.array([0.0, 0.0, 0.0])

     
# USB 카메라로 실시간 비디오 캡처
#cap = cv2.VideoCapture(2)  # 2번 디바이스 (USB 카메라)


class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        
        # QoS 프로파일 생성 방식 변경
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            depth=10
        )

        # 구독자 설정
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera_raw',  # 실제 카메라 토픽으로 교체
            self.listener_callback,
            qos_profile=qos_profile 
        )
        
        self.get_logger().info("Aruco Detection Node Started")
        
        self.zones = {  # 구역을 나타내는 값들
            'initial_pos': 0,   # 최초 위치
            'picking_pos': 1,   # 박스 위치 and 바스켓 위치
            'loading_pos': 2    # 하역장 위치
        }
        self.zone = 0  # 기본 구역 값
        #self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)  # ArUco 딕셔너리

        # self.publisher_ = self.create_publisher(
        #     Int32, 
        #     'aruco_zone', 
        #     qos_profile=qos_profile 
        # ) 
        
        # timer_period = 1 / 4  # 콜백 주기: 4Hz
        # self.timer = self.create_timer(timer_period, self.timer_callback)  # 주기적 실행을 위한 타이머
        # self.i = 0  # 퍼블리시 횟수 카운터
        
        # 상대 위치를 위한 토픽 (커스텀 메시지)
        self.relative_position_pub = self.create_publisher(
            RelativePosition, 
            'marker_relative_position',
            qos_profile=qos_profile 
        )
        
        #self.relative_position_client = self.create_client(RelativePosition, 'send_relative_position')        
        # while not self.relative_position_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service "send_relative_position" not available, waiting...')

        self.get_logger().info("ArucoDetection Node Initialized")
         
        
    def listener_callback(self, image_data):
        """
        이미지 데이터를 구독했을 때 호출되는 콜백 함수
        """
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")  # ROS 이미지를 OpenCV 이미지로 변환
        except CvBridgeError as e:
            print(e)
        #print(type(image_data), print(type(cv_image))) # sensor_msgs.msg.Image, np.array 타입 출력
                
        self.aruco_detect_markers(cv_image)
        

    # def send_to_server(self, marker_id, relative_rvec, relative_tvec):
    #     # 서버에 상대 좌표 데이터를 전송
    #     request = RelativePosition.Request()
    #     request.marker_id = marker_id
    #     request.relative_rvec = relative_rvec.tolist()
    #     request.relative_tvec = relative_tvec.tolist()

    #     self.get_logger().info(f"Sending data to server: Marker ID {marker_id}")
    #     future = self.relative_position_client.call_async(request)

    #     def response_callback(future_result):
    #         try:
    #             response = future_result.result()
    #             if response.success:
    #                 self.get_logger().info(f"Successfully sent data for Marker ID {marker_id}")
    #             else:
    #                 self.get_logger().error(f"Failed to send data for Marker ID {marker_id}")
    #         except Exception as e:
    #             self.get_logger().error(f"Service call failed: {str(e)}")
        
    #     future.add_done_callback(response_callback)

    def find_zone(self, image, corners, ids):
        """
        특정 ArUco ID(예: 9번)를 탐지하고 해당 위치 구역을 반환
        """
        zone = 0  # 기본 구역
        corners_id = 0  # ArUco 코너 ID 초기값
        found_corner = None  # 발견된 코너 초기값
        for id in ids:  # 탐지된 모든 ID 순회
            if id == 9:  # 목표 ID가 9일 경우
                found_corner = corners[corners_id][0]  # 해당 코너 추출
                zone = found_corner
                break  # 탐지 종료
            else:
                corners_id += 1  # 다음 ID로 이동
                continue

        return zone, found_corner  # 구역과 코너 좌표 반환
    
    def timer_callback(self):
        """
        주기적으로 호출되는 타이머 콜백 함수. 구역 데이터를 퍼블리시.
        """
    
        publish_zone = Int32()
        publish_zone.data = self.zone  # 현재 구역 값 설정
        self.publisher_.publish(publish_zone)  # 퍼블리시
        self.get_logger().info('detect_aruco: zone "%d" published to /aruco_zone.' % publish_zone.data)  # 로그 출력
        self.i += 1  # 카운터 증가

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
            angle_deg: 상대 방향 벡터의 각도 (도 단위, 기준: x축)
        """
        # 기준 마커의 회전 행렬 계산
        R_base, _ = cv2.Rodrigues(base_rvec)  # 기준 마커의 회전 행렬
        R_base_inv = np.linalg.inv(R_base)    # 회전 행렬의 역행렬

        # 타겟 마커의 이동 벡터를 기준 좌표계로 변환
        tvec_relative = R_base_inv @ (target_tvec.T - base_tvec.T)

        # x, z 성분만 추출
        tvec_relative_xz = tvec_relative[[0, 2]]  # x, z 좌표 (상대 이동)
        #tvec_relative_xz = tvec_relative[:2]  # x, z 좌표 (상대 이동)
        distance = np.linalg.norm(tvec_relative_xz)  # 거리 계산
        direction_vector = tvec_relative_xz / distance  # 방향 벡터 (단위 벡터)

        # x, z를 기반으로 각도 계산 (atan2 사용)
        angle_rad = np.arctan2(tvec_relative_xz[1], tvec_relative_xz[0])  # 라디안 단위
        angle_deg = np.degrees(angle_rad)  # 도 단위 변환

        # scipy를 사용하여 쿼터니언으로 변환
        r = R.from_euler('z', angle_rad)  # 'z'축 회전만 고려
        quaternion = r.as_quat()

        return tvec_relative_xz.flatten(), distance, angle_rad, quaternion

    # 아루코 마커 상대좌표 계산
    def aruco_detect_markers(self, cv_image):
        self.get_logger().info('2...')

        # 실시간 비디오 캡처에서 계속 반복되므로, 첫 번째 마커만 처리하도록 설정
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_params)

        if ids is not None:
            print(f"[INFO] Detected markers: {ids.flatten()}")
            self.get_logger().info('3...')

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            self.get_logger().info('4...')

            # 기준 마커 (ID 2)와 타겟 마커 (ID 15) 찾기
            base_rvec, base_tvec = None, None
            target_rvec, target_tvec = None, None

            for i, marker_id in enumerate(ids.flatten()):
                self.get_logger().info('5...')
                if marker_id == 2:  # 기준 마커
                    base_rvec, base_tvec = rvecs[i], tvecs[i]
                elif marker_id == 15:  # 타겟 마커
                    target_rvec, target_tvec = rvecs[i], tvecs[i]

                if base_rvec is not None and base_tvec is not None and target_rvec is not None and target_tvec is not None:
                    relative_tvec, distance, angle_rad, quarternion = self.calculate_relative_position(
                        base_rvec, base_tvec, target_rvec, target_tvec
                    )           
                    self.get_logger().info('6...') 
                    # 메시지 생성
                    msg = RelativePosition()
                    msg.marker_id = 15
                    msg.x = relative_tvec[0]
                    #msg.y = 0.0  # y값은 사용하지 않음
                    msg.z = relative_tvec[1]
                    msg.distance = distance        
                    
                    print((f"Type of angle_rad: {type(angle_rad)}"))
                    for value in angle_rad:
                        print(f"angle_rad: {value:.6f}")                    
  
                    # angle_rad가 numpy.ndarray일 경우 첫 번째 값만 추출하여 할당
                    if isinstance(angle_rad, np.ndarray):
                        msg.angle = angle_rad[0]  # 첫 번째 값만 사용 (혹은 필요한 값 선택)
                    else:
                        msg.angle = angle_rad  # 이미 float이면 그대로 사용
                        
                    # Quaternion 객체 생성
                    quarternion = Quaternion()
                    # msg.quaternion 필드에 할당
                    msg.quaternion = quarternion
                    
                    self.get_logger().info(f"marker_id: {msg.marker_id}")
                    self.get_logger().info(f"relative position: {msg.z}, {msg.x}")
                    self.get_logger().info(f"distance: {msg.distance}")
                    self.get_logger().info(f"angle (radians): {msg.angle:.6f}")  # angle을 소수점 6자리까지 포맷
                    self.get_logger().info(f"quaternion: {msg.quaternion}")

                    # 메시지 발행
                    self.relative_position_pub.publish(msg)
                

            # ArUco 마커와 축을 이미지에 그리기
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            # ArUco 축을 첫 번째로만 그리도록 조건 추가
            # for i in range(len(ids)):
            #     cv2.aruco.drawAxis(cv_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length)

            # 처리된 이미지를 화면에 표시
            cv2.imshow("ArUco Markers", cv_image)
            cv2.waitKey(1)  # OpenCV 창을 업데이트
        else:
            print(f"[WARN] 기준 마커(ID 2) 또는 타겟 마커(ID 15)를 찾을 수 없음.")        

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
        aruco_detection.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
