import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import math
from virtual_factory_if.srv import GoalLocation
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
#from tf2_geometry_msgs.tf2_geometry_msgs import toMsg
from scipy.spatial.transform import Rotation as R


#import tf_transformations as tft  # 기존 라이브러리를추가!

class AMRNode(Node):
    def __init__(self):
        super().__init__('amr_node')

        # QoS 프로파일 생성 방식 변경
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            depth=10
        )
        
        # 이동 중지 플래그 초기화
        self.stop_requested = False

        # 로봇의 현재 위치
        self.current_position = Point()

        # 아루코 마커의 상대 좌표 (기준 좌표로부터의 거리 및 벡터 정보)
        self.aruco_marker_position = Point()
        # Initialize quaternion for orientation
        self.quaternion = Quaternion()  # Initialize quaternion
        # Other initializations (e.g., current_position, PID, etc.)
        self.current_position = Point()
        self.current_quaternion = None
        self.current_orientation = None  # 현재 로봇 방향 초기화
  
        # PID 제어 변수 초기화
        self.kp = 1.0  # 비례 게인
        self.ki = 0.0  # 적분 게인
        self.kd = 0.5  # 미분 게인

        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        # 이동 명령을 발행하는 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 초기 위치 퍼블리셔
        self.initial_pose_pub = self.create_publisher(
            PoseStamped,
            '/initialpose',
            qos_profile=qos_profile 
        )
        
        # 상태 메시지를 퍼블리시하는 퍼블리셔 추가
        self.status_pub = self.create_publisher(
            String,
            'status_topic',
            qos_profile=qos_profile
        )

        # odometry 위치 정보를 구독
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odom',  # 또는 '/amcl_pose'로 변경 가능
            self.pose_callback,
            qos_profile=qos_profile 
        )
        
        # 아루코 마커의 상대 좌표를 받는 토픽 구독
        self.goal_sub = self.create_service(
            GoalLocation,  # 상대적인 거리 및 벡터 정보를 담고 있는 메시지
            '/goal_position',  # 아루코 마커의 상대 좌표를 받는 토픽
            self.aruco_marker_callback
        )
        
        # Stop 명령을 수신하는 서브스크라이버 생성
        self.stop_subscriber = self.create_subscription(
            String,
            'amr_control_topic',
            self.amr_control_callback,
            qos_profile=qos_profile
        )        

        # 목표 좌표 계산 및 전송
        self.calculate_goal_from_marker()

    def set_initial_pose(self, x, y, yaw):
        """초기 위치를 설정하고 퍼블리시하는 함수"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'  # 'map' 대신 'odom' 사용
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.w = 1.0  # 'yaw' 계산은 이니셜 설정에서 직접 다룰 수 있음

        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"Initial pose set to x={x}, y={y}, yaw={yaw}")

    def aruco_marker_callback(self, request, response):
        self.get_logger().info('목표 마커: %s' % request.marker_id)

        # 받은 x, y 좌표와 쿼터니언 값 출력
        self.get_logger().info(f'Received pose: x={request.z}, y={request.x}')
        self.get_logger().info(f"Received distance: {request.distance.data}")
        self.get_logger().info(f'Received quaternion: x={request.quaternion.x}, y={request.quaternion.y}, z={request.quaternion.z}, w={request.quaternion.w}')
        
        """아루코 마커의 상대 좌표를 받는 콜백 함수"""
        # 아루코마커의 z,x가 로봇좌표계에서는 x,y 
        self.aruco_marker_position.x = request.z
        self.aruco_marker_position.y = request.x
        self.quaternion = request.quaternion
        
        # 현재 목표로 하는 마커 ID 저장
        self.current_marker_id = request.marker_id
            
        # 아루코 마커의 상대 좌표를 받았으므로 목표 좌표 계산
        self.calculate_goal_from_marker()
 
        # 응답 설정
        response.success = True
        response.message = "Quaternion received successfully"
        return response
   
    def calculate_goal_from_marker(self):
        """아루코 마커에서 얻은 상대 좌표를 바탕으로 목표 좌표를 계산"""
        # 로봇 위치와 아루코 마커의 상대적인 벡터 계산
        # 아루코마커의 z,x가 로봇좌표계에서는 x,y 
        delta_x = self.aruco_marker_position.z  # 기준 좌표로부터의 상대적 x 거리
        delta_y = self.aruco_marker_position.x  # 기준 좌표로부터의 상대적 y 거리
        #delta_y = -self.aruco_marker_position.x  # - 를 붙여야 할 수도 있음. 직접 재봐야 함. 

        # 목표 좌표는 로봇 위치와 아루코 마커 위치 간의 벡터 차이를 기반으로 설정
        goal_x = self.current_position.x + delta_x
        goal_y = self.current_position.y + delta_y

        # 목표 좌표 전송
        self.navigate_by_cmd_vel(goal_x, goal_y)
        #self.move_to_goal(goal_x, goal_y)
  
    def navigate_by_cmd_vel(self, goal_x, goal_y):
        
         # 이동 중인지 여부 확인
        if self.stop_requested:
            # 이동 중지 명령을 받았으므로 속도를 0으로 설정하고 리턴
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info("Movement stopped due to stop command.")
            return
        
        """cmd_vel을 사용하여 목표로 이동"""
        move_cmd = Twist()

        # 목표 위치까지의 거리 및 각도 계산
        delta_x = goal_x - self.current_position.x
        delta_y = goal_y - self.current_position.y

        # 목표 방향 (라디안)
        target_angle = math.atan2(delta_y, delta_x)
        
        # 현재 로봇의 방향 (현재 orientation 값을 yaw로 변환)
        if self.current_orientation is not None:
            quaternion_list = [
                self.current_orientation.x,
                self.current_orientation.y,
                self.current_orientation.z,
                self.current_orientation.w
            ]
            # Quaternion을 Euler 각도로 변환하여 yaw를 추출
            current_rotation = R.from_quat(quaternion_list)
            current_yaw = current_rotation.as_euler('xyz')[2]  # Euler 각도에서 yaw 값 추출
        else:
            # 오류 처리 또는 기본값 설정
            current_yaw = 0.0  # 기본 yaw 값

        # 목표 yaw (목표 지점의 방향)
        target_yaw = target_angle

        # 각도 오차 계산
        angle_error = target_yaw - current_yaw
        # 각도를 [-pi, pi] 범위로 정규화
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 각속도 제어 (P제어)
        if abs(angle_error) > 0.1:  # 목표로 방향 정렬
            move_cmd.angular.z = min(max(angle_error * self.kp, -0.5), 0.5)
            move_cmd.linear.x = 0.0  # 회전 중에는 전진 속도를 0으로 설정
        else:
            # 방향이 맞으면 전진 시작
            move_cmd.angular.z = 0.0
            distance = math.sqrt(delta_x**2 + delta_y**2)
            if distance > 0.1:  # 목표 도달 허용 오차
                move_cmd.linear.x = min(distance * self.kp, 0.2)  # 최대 속도 제한
            else:
                # 목표 도달 시 속도를 0으로 설정
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
                self.get_logger().info(f"목표 도달: x={goal_x}, y={goal_y}")

                # 목표에 도달했으므로 상태 메시지 퍼블리시
                if self.current_marker_id is not None:
                    status_msg = String()
                    status_msg.data = f'arrived at marker {self.current_marker_id}'
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(f"Published status: {status_msg.data}")

                return  # 더 이상 이동 명령을 발행하지 않음

        # cmd_vel 퍼블리시
        self.cmd_vel_pub.publish(move_cmd)
        self.get_logger().info(f"목표로 이동 중: x={goal_x}, y={goal_y}, "
                            f"linear.x={move_cmd.linear.x}, angular.z={move_cmd.angular.z}")

    def move_to_goal(self, goal_x, goal_y, quaternion):
        """목표 좌표와 방향으로 로봇을 이동시키는 함수"""
        # 로컬 좌표계에서 목표로 이동
        error_x = goal_x - self.current_position.x
        error_y = goal_y - self.current_position.y

        # PID 제어
        control_signal_x = self.pid_control(error_x, self.previous_error_x, self.integral_x, 'x')
        control_signal_y = self.pid_control(error_y, self.previous_error_y, self.integral_y, 'y')

        # 이동 명령 생성
        move_cmd = Twist()

        # 목표에 가까워지면 속도를 줄임
        if abs(error_x) < 0.1 and abs(error_y) < 0.1:
            move_cmd.linear.x = 0.0  # 목표에 가까우면 속도를 0으로 설정
            move_cmd.angular.z = 0.0
            self.get_logger().info(f"목표에 도달: x={goal_x}, y={goal_y}")
        else:
            # X축과 Y축에 대한 속도 계산
            move_cmd.linear.x = min(0.5, control_signal_x)  # 속도 제한
            move_cmd.angular.z = min(0.5, control_signal_y)  # 회전 속도 설정

        # 방향 정보 (quaternion)도 이동 명령에 포함시킴
        move_cmd.orientation.x = quaternion.x
        move_cmd.orientation.y = quaternion.y
        move_cmd.orientation.z = quaternion.z
        move_cmd.orientation.w = quaternion.w

        # 로봇의 속도 명령을 퍼블리시
        self.cmd_vel_pub.publish(move_cmd)

        self.get_logger().info(f"PID 제어 신호: x={control_signal_x}, y={control_signal_y}, "
                                f"Orientation: x={quaternion.x}, y={quaternion.y}, "
                                f"z={quaternion.z}, w={quaternion.w}")

    def pid_control(self, error, previous_error, integral, axis):
        """PID 제어를 계산하는 함수"""
        # 비례 항 (P)
        proportional = self.kp * error

        # 적분 항 (I)
        integral += error
        integral_term = self.ki * integral

        # 미분 항 (D)
        derivative = error - previous_error
        derivative_term = self.kd * derivative

        # PID 제어 신호 계산
        control_signal = proportional + integral_term + derivative_term

        # PID 결과 반환 및 업데이트
        if axis == 'x':
            self.previous_error_x = error
            self.integral_x = integral
        elif axis == 'y':
            self.previous_error_y = error
            self.integral_y = integral

        return control_signal

    def pose_callback(self, msg):
        """Odometry 메시지에서 위치 정보 가져오기"""
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

        # 방향 정보 (쿼터니언) 가져오기
        self.current_orientation = msg.pose.pose.orientation

        self.get_logger().info(f"AMR 위치 업데이트: "
                            f"Position: x={self.current_position.x:.3f}, "
                            f"y={self.current_position.y:.3f}, "
                            f"z={self.current_position.z:.3f}")
        self.get_logger().info(f"AMR 방향 업데이트: "
                            f"Orientation: x={self.current_orientation.x:.3f}, "
                            f"y={self.current_orientation.y:.3f}, "
                            f"z={self.current_orientation.z:.3f}, "
                            f"w={self.current_orientation.w:.3f}")
        
    def amr_control_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "stop":
            self.stop_requested = True
            self.get_logger().info("Received stop command. Stopping movement.")
        elif command == "resume":
            self.stop_requested = False
            self.get_logger().info("Received resume command. Resuming movement.")

def main():
    rclpy.init()
    
    # ROS2 노드 생성
    amr_node = AMRNode()
    
    # 노드 실행
    rclpy.spin(amr_node)
    
    # 노드를 명시적으로 파괴 (선택 사항, 노드가 자동으로 파괴될 수 있음)
    amr_node.destroy_node()
    
    # ROS2 종료
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
