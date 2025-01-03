import sys
from PyQt5.QtGui import QPixmap, QImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from PyQt5.QtCore import Qt, QTimer, QTime, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QGroupBox, QButtonGroup, QStackedWidget, QComboBox)
from rclpy.qos import QoSProfile, ReliabilityPolicy
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import ssl
import serial
import threading
from rclpy.executors import MultiThreadedExecutor
import cv2
import numpy as np
import json
from virtual_factory_if.msg import RelativePosition 
from virtual_factory_if.srv import GoalLocation  
from std_msgs.msg import Int32, Float32, String 

class LoginWindow(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.error_label = QLabel()  # 에러 메시지 레이블을 초기화

        # Main Layout
        layout = QVBoxLayout()
        layout.setContentsMargins(100, 50, 100, 50)  # Add margins for spacing
        layout.setSpacing(50)  # Add spacing between widgets

        # Titleprocess_frame
        self.title = QLabel("Robot Control System\n\n\nLogin")
        self.title.setFixedHeight(300)
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet("font-size: 36px; font-weight: bold; margin-bottom: 30px;")
        layout.addWidget(self.title, alignment=Qt.AlignTop)

        # Input Fields Layout
        fields_layout = QHBoxLayout()
        fields_layout.setSpacing(30)  # Add spacing between input fields

        # ID Field
        self.id_layout = QVBoxLayout()
        self.id_label = QLabel("E-mail")
        self.id_label.setStyleSheet("font-size: 30px;")
        self.id_input = QLineEdit()
        self.id_input.setPlaceholderText("Enter your E-mail")
        self.id_input.setFixedHeight(50)  # Make input fields taller
        self.id_input.setStyleSheet("font-size: 20px; padding: 5px;")
        self.id_layout.addWidget(self.id_label)
        self.id_layout.addWidget(self.id_input)

        # Password Field
        self.password_layout = QVBoxLayout()
        self.password_label = QLabel("Password")
        self.password_label.setFixedHeight(50)
        self.setStyleSheet("font-size: 30px;")
        self.password_input = QLineEdit()
        # self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setPlaceholderText("Enter your Password")
        self.password_input.setFixedHeight(50)
        self.password_input.setStyleSheet("font-size: 20px; padding: 5px;")
        self.password_layout.addWidget(self.password_label)
        self.password_layout.addWidget(self.password_input)

        # Add ID and Password layouts to the horizontal layout
        fields_layout.addLayout(self.id_layout)
        fields_layout.addLayout(self.password_layout)

        # Add fields layout to the main layout
        layout.addLayout(fields_layout, stretch=2)

        # Error Label
        self.error_label.setStyleSheet("color: red; font-size: 20px;")
        self.error_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.error_label)  # 에러 메시지를 레이아웃에 추가

        # Login Button
        login_button = QPushButton("Login")
        login_button.setStyleSheet(
            "background-color: green; color: white; font-size: 30px; padding: 15px;"
        )
        login_button.setFixedWidth(500)  # Button width
        login_button.setFixedHeight(100)  # Button height
        login_button.clicked.connect(self.login)
        layout.addWidget(login_button, alignment=Qt.AlignCenter)

        self.setLayout(layout)

    def login(self):
        user_id = self.id_input.text()
        password = self.password_input.text()

        if user_id == "jsh10198@naver.com" and password == "password":
            self.stacked_widget.setCurrentIndex(1)
            self.error_label.setText("")  # 에러 메시지를 비움
        else:
            self.error_label.setText("Invalid E-mail or Password")  # 에러 메시지를 업데이트

class MainApplication(QStackedWidget):
    def __init__(self):
        super().__init__()
        
        # QoS 프로파일 생성
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            depth=10
        )

        self.setWindowTitle("Robot Control System")
        self.setGeometry(100, 100, 1600, 1000)
        self.node = Node("gui_node")  # ROS2 노드 생성
        self.bridge = CvBridge()  # CvBridge 초gui_server2기화
        self.publisher_conveyor = self.node.create_publisher(String, "conveyor_test", 10)
        self.publisher_learning = self.node.create_publisher(String, "learning_test", 10)
        self.publisher_confirm = self.node.create_publisher(String, 'target_counts', 10)
        self.timer = QTimer()
        self.elapsed_time = QTime(0, 0, 0)
        self.timer.timeout.connect(self.update_task_time)
        self.robot_status = "Idle"
        self.coord_x = ""
        self.coord_y = ""
        self.coord_z = ""
        self.coord_rx = ""
        self.coord_ry = ""
        self.coord_rz = ""
        
        # status 토픽 Subscribe
        self.subscription_status = self.node.create_subscription(
            String, 'status_topic', self.status_callback, 10
        )        
        
        # coordinate 토픽 Subscribe
        self.subscription_coordinate = self.node.create_subscription(
            RelativePosition, 
            'marker_relative_position', 
            self.coordinate_callback, 
            qos_profile=qos_profile
        )               
        
        # webcam_image 토픽 Subscribe
        self.subscription_world = self.node.create_subscription(
            Image, 
            'aruco_detected_image',
            self.world_eye_image_callback, 
            qos_profile=qos_profile 
        )                       
      
        # hand_image 토픽 Subscribe
        self.subscription_hand = self.node.create_subscription(
            Image, 
            'hand_image', 
            self.hand_eye_image_callback, 
            qos_profile=qos_profile 
        )
        
        # 목표 위치를 요청할 수 있는 클라이언트 생성
        self.goal_position = self.node.create_client(
            GoalLocation,
            '/goal_position',
            qos_profile=qos_profile
        )
        
        # gripper_image 토픽 Subscribe
        # self.subscription_hand = self.node.create_subscription(
        #     Image, 'gripper_image', self.hand_eye_image_callback, 10
        # )
                
        self.world_timer = QTimer()
        self.world_timer.timeout.connect(self.spin_ros_world)  # ROS 이벤트 처리
        self.world_timer.start(10)  # 10ms 간격으로 spin 호출
        
        self.hand_timer = QTimer()
        self.hand_timer.timeout.connect(self.spin_ros_hand)  # ROS 이벤트 처리
        self.hand_timer.start(10)  # 10ms 간격으로 spin 호출

        # 로그인 창 추가
        self.login_window = LoginWindow(self)
        self.addWidget(self.login_window)

        # 메인 애플리케이션 UI 구성
        main_widget = QWidget()
        main_layout = QVBoxLayout(main_widget)

        # Top Section
        top_layout = QHBoxLayout()

        world_eye_layout = QVBoxLayout()
        world_eye_label = QLabel("World Eye")
        world_eye_label.setAlignment(Qt.AlignCenter)
        world_eye_label.setFixedHeight(30)
        world_eye_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        self.world_eye_display = QLabel()
        self.world_eye_display.setStyleSheet("border: 2px solid black; background-color: #E8F5E9;")
        self.world_eye_display.setFixedSize(750, 500)
        world_eye_layout.addWidget(world_eye_label)
        world_eye_layout.addWidget(self.world_eye_display)

        hand_eye_layout = QVBoxLayout()
        hand_eye_label = QLabel("Hand Eye")
        hand_eye_label.setAlignment(Qt.AlignCenter)
        hand_eye_label.setFixedHeight(30)
        hand_eye_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        self.hand_eye_display = QLabel()
        self.hand_eye_display.setStyleSheet("border: 2px solid black; background-color: #E3F2FD;")
        self.hand_eye_display.setFixedSize(750, 500)
        hand_eye_layout.addWidget(hand_eye_label)
        hand_eye_layout.addWidget(self.hand_eye_display)

        top_layout.addLayout(world_eye_layout)
        top_layout.addLayout(hand_eye_layout)
        main_layout.addLayout(top_layout)

        # Bottom Section
        bottom_layout = QHBoxLayout()
        control_status_layout = QVBoxLayout()
        control_status_layout.addWidget(self.create_robot_button_group())
        control_status_layout.addWidget(self.robot_status_group())
        bottom_layout.addLayout(control_status_layout)

        # Conveyor and Learning Vertical Group
        conveyor_learning_layout = QVBoxLayout()
        conveyor_learning_layout.addWidget(self.create_conveyor_button_group())
        conveyor_learning_layout.addWidget(self.create_learning_button_group())
        bottom_layout.addLayout(conveyor_learning_layout)

        bottom_layout.addWidget(self.create_manipulator_button_group())
        main_layout.addLayout(bottom_layout)

        self.setLayout(main_layout)
        self.set_all_buttons_enabled(False)  # 모든 버튼 비활성화
        self.confirm_button.setEnabled(True)  # 초기 상태에서 robot_start만 활성화

        self.addWidget(main_widget)
        self.setCurrentIndex(0)  # 초기화면은 로그인 창
        
    def create_display_group(self, title, width, height):
        layout = QVBoxLayout()
        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setFixedHeight(30)
        label.setStyleSheet("font-size: 20px; font-weight: bold;")
        display = QLabel()
        display.setFixedSize(width, height)
        display.setStyleSheet("border: 2px solid black;")
        layout.addWidget(label)
        layout.addWidget(display)
        return layout

    def create_robot_button_group(self):
        group_box = QGroupBox("Control")
        group_box.setFixedWidth(500)
        layout = QVBoxLayout()

        # Start 버튼
        self.robot_start_button = QPushButton("Play")
        self.robot_start_button.setFixedHeight(50)
        self.robot_start_button.clicked.connect(self.robot_start)
        layout.addWidget(self.robot_start_button)

        # Pause와 Resume 버튼
        pause_resume_layout = QHBoxLayout()
        self.robot_pause_button = QPushButton("Pause")
        self.robot_pause_button.setFixedHeight(50)
        self.robot_pause_button.clicked.connect(self.robot_pause)
        pause_resume_layout.addWidget(self.robot_pause_button)

        self.robot_resume_button = QPushButton("Resume")
        self.robot_resume_button.setFixedHeight(50)
        self.robot_resume_button.clicked.connect(self.robot_resume)
        pause_resume_layout.addWidget(self.robot_resume_button)
        layout.addLayout(pause_resume_layout)

        # Stop 버튼
        stop_reset_layout = QHBoxLayout()
        self.robot_stop_button = QPushButton("Stop")
        self.robot_stop_button.setFixedHeight(50)
        self.robot_stop_button.setStyleSheet("background-color: red; color: white;")
        self.robot_stop_button.clicked.connect(self.robot_stop)
        stop_reset_layout.addWidget(self.robot_stop_button)

        # Reset 버튼
        self.robot_reset_button = QPushButton("Reset")
        self.robot_reset_button.setFixedHeight(50)
        self.robot_reset_button.clicked.connect(self.robot_reset)
        stop_reset_layout.addWidget(self.robot_reset_button)
        layout.addLayout(stop_reset_layout)
        
        group_box.setLayout(layout)
        return group_box
    
    def robot_status_group(self):
        group_box = QGroupBox("Status")
        group_box.setFixedWidth(500)
        layout = QVBoxLayout()
    
        # Robot Status Label
        self.robot_status_label = QLabel(f"Status: {self.robot_status}")
        self.robot_status_label.setFixedHeight(25)
        self.robot_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.robot_status_label)
        
        # Task Time Label
        self.task_time_label = QLabel("Task Time: 00:00:00")
        self.task_time_label.setFixedHeight(25)
        self.task_time_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.task_time_label)
        
        # Robot Coordinate Label
        self.robot_coordinate_label = QLabel(f"X( {self.coord_x}mm )   Y( {self.coord_y}mm )   Z( {self.coord_z}mm )\nRX( {self.coord_rx}deg )   RY( {self.coord_ry}deg )   RZ( {self.coord_rz}deg )")
        self.robot_coordinate_label.setFixedHeight(50)
        self.robot_coordinate_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.robot_coordinate_label)
        
        group_box.setLayout(layout)
        return group_box

    def create_conveyor_button_group(self):
        group_box = QGroupBox("Conveyor")
        group_box.setFixedWidth(500)
        layout = QVBoxLayout()

        # Start/Stop 버튼
        self.conveyor_start_stop_toggle = QPushButton("Conveyor Start")
        self.conveyor_start_stop_toggle.setFixedHeight(50)
        self.conveyor_start_stop_toggle.setCheckable(True)
        self.conveyor_start_stop_toggle.clicked.connect(self.conveyor_start_stop)
        layout.addWidget(self.conveyor_start_stop_toggle)

        group_box.setLayout(layout)
        return group_box

    def create_learning_button_group(self):
        group_box = QGroupBox("Learning")
        group_box.setFixedWidth(500)
        layout = QVBoxLayout()

        # Start/Stop 버튼
        self.learning_start_stop_toggle = QPushButton("Learning Start")
        self.learning_start_stop_toggle.setFixedHeight(50)
        self.learning_start_stop_toggle.setCheckable(True)
        self.learning_start_stop_toggle.clicked.connect(self.learning_start_stop)
        layout.addWidget(self.learning_start_stop_toggle)

        group_box.setLayout(layout)
        return group_box

    def create_manipulator_button_group(self):
        group_box = QGroupBox("Job")
        group_box.setFixedWidth(500)
        layout = QVBoxLayout()

        # 작업 선택 드롭다운
        input_layout = QHBoxLayout()
        self.work_combobox = QComboBox()
        self.work_combobox.addItems(["Nothing",
                                     "Red*2,   Blue*1,   Goal1",
                                     "Red*1,   Blue*2,   Goal2",
                                     "Red*1,   Goal3"])
        self.work_combobox.setFixedHeight(50)
        input_layout.addWidget(self.work_combobox)
        layout.addLayout(input_layout)
        
        self.confirm_button = QPushButton("Confirm")
        self.confirm_button.setFixedHeight(50)
        self.confirm_button.clicked.connect(self.confirm_play)
        layout.addWidget(self.confirm_button)

        group_box.setLayout(layout)
        return group_box
    
    def set_all_buttons_enabled(self, enabled):
        """모든 버튼을 활성화/비활성화"""
        self.robot_start_button.setEnabled(enabled)
        self.robot_pause_button.setEnabled(enabled)
        self.robot_resume_button.setEnabled(enabled)
        self.robot_stop_button.setEnabled(enabled)
        self.robot_reset_button.setEnabled(enabled)

    def robot_start(self):
        # self.robot_status = "Running"
        self.timer.start(1000)
        # self.update_status()
        self.set_all_buttons_enabled(True)
        self.robot_start_button.setEnabled(False)
        self.robot_resume_button.setEnabled(False)
        self.robot_reset_button.setEnabled(False)
        self.node.get_logger().warning("Control Play")

    def robot_pause(self):
        # self.robot_status = "Paused"
        self.timer.stop()
        # self.update_status()
        self.robot_pause_button.setEnabled(False)
        self.robot_resume_button.setEnabled(True)
        self.node.get_logger().warning("Control Pause")

    def robot_resume(self):
        # self.robot_status = "Running"
        self.timer.start(1000)
        # self.update_status()
        self.robot_pause_button.setEnabled(True)
        self.robot_resume_button.setEnabled(False)
        self.robot_start_button.setEnabled(False)
        self.node.get_logger().warning("Control Resume")

    def robot_stop(self):
        # self.robot_status = "Stopped"
        self.timer.stop()
        # self.update_status()
        self.set_all_buttons_enabled(False)
        self.robot_start_button.setEnabled(False)
        self.robot_reset_button.setEnabled(True)
        self.conveyor_start_stop_toggle.setChecked(False)
        self.conveyor_start_stop_toggle.setText("Conveyor Start")
        msg_con = String()
        msg_con.data = "Conveyor Stop"
        self.publisher_conveyor.publish(msg_con)
        self.learning_start_stop_toggle.setText("Learning Start")
        msg_lea = String()
        msg_lea.data = "Learning Stop"
        self.publisher_conveyor.publish(msg_lea)
        self.learning_start_stop_toggle.setChecked(False)
        self.conveyor_start_stop_toggle.setEnabled(False)
        self.learning_start_stop_toggle.setEnabled(False)
        self.node.get_logger().warning("Control Stop")

    def robot_reset(self):
        # self.robot_status = "Reset"
        self.timer.stop()
        self.elapsed_time = QTime(0, 0, 0)
        self.update_task_time()
        # self.update_status()
        self.set_all_buttons_enabled(False)
        self.robot_start_button.setEnabled(False)
        self.confirm_button.setEnabled(True)
        self.conveyor_start_stop_toggle.setEnabled(True)
        self.learning_start_stop_toggle.setEnabled(True)
        self.node.get_logger().warning("Control Reset")

    def conveyor_start_stop(self):
        if self.conveyor_start_stop_toggle.isChecked():
            self.conveyor_start_stop_toggle.setText("Conveyor Stop")
            msg = String()
            msg.data = "Conveyor Start"  # ROS 메시지 설정
            self.publisher_conveyor.publish(msg)  # 메시지 발행
            self.node.get_logger().warning("Conveyor Start")
        else:
            self.conveyor_start_stop_toggle.setText("Conveyor Start")
            msg = String()
            msg.data = "Conveyor Stop"
            self.publisher_conveyor.publish(msg)
            self.node.get_logger().warning("Conveyor Stop")
            
    def learning_start_stop(self):
        if self.learning_start_stop_toggle.isChecked():
            self.learning_start_stop_toggle.setText("Learning Stop")
            msg = String()
            msg.data = "Learning Start"
            self.publisher_learning.publish(msg)
            self.node.get_logger().warning("Learning Start")
        else:
            self.learning_start_stop_toggle.setText("Learning Start")
            msg = String()
            msg.data = "Learning Stop"
            self.publisher_learning.publish(msg)
            self.node.get_logger().warning("Learning Stop")

    def confirm_play(self):
        selected_text = self.work_combobox.currentText()
        self.confirm_button.setEnabled(False)
        self.robot_start_button.setEnabled(True)
        self.node.get_logger().warning(f"{selected_text} Confirm")
        # 선택된 텍스트를 JSON 데이터로 변환
        data = {}
        items = selected_text.split(",")  # ','로 항목 분리
        for item in items:
            parts = item.strip().split("*")  # '*'로 색상과 개수 분리
            if len(parts) == 2:  # 유효한 항목만 처리
                color, count = parts[0].strip().upper(), int(parts[1].strip())
                data[color] = count
        # JSON 데이터로 변환
        json_data = json.dumps(data)
        # ROS2 메시지 퍼블리시
        message = String()
        message.data = json_data
        self.node.get_logger().warning(f"Publishing JSON: {json_data}")
        self.publisher_confirm.publish(message)
        
    def update_task_time(self):
        self.elapsed_time = self.elapsed_time.addSecs(1)
        self.task_time_label.setText(f"Task Time: {self.elapsed_time.toString('hh:mm:ss')}")

    def update_status(self, msg):
        self.status = msg
        self.robot_status_label.setText(f"Status: {self.status}")
        
    def update_coordinate(self, msg):       
        
        # 정수 거리 값을 계산하여 3자리로 표시
        x_int = int(abs(msg.x * 1000))
        y_int = int(abs(msg.y * 1000))
        z_int = int(abs(msg.z * 1000))

        # 3자리 숫자로 포맷팅
        x_formatted = f"{x_int:03}"
        y_formatted = f"{y_int:03}"
        z_formatted = f"{z_int:03}"
        
        roll, pitch, yaw = msg.rx, msg.ry, msg.rz

        # 각도를 계산하고 3자리로 포맷팅 (rx, ry, rz)
        rx_angle = int(abs(np.degrees(roll)))  # 라디안 → 도
        ry_angle = int(abs(np.degrees(pitch)))
        rz_angle = int(abs(np.degrees(yaw)))

        rx_formatted = f"{rx_angle:03}"
        ry_formatted = f"{ry_angle:03}"
        rz_formatted = f"{rz_angle:03}"

        self.coord_x = x_formatted
        self.coord_y = y_formatted
        self.coord_z = z_formatted
        self.coord_rx = rx_formatted
        self.coord_ry = ry_formatted
        self.coord_rz = rz_formatted
        self.robot_coordinate_label.setText(f"X( {self.coord_x}mm )   Y( {self.coord_y}mm )   Z( {self.coord_z}mm )\nRX( {self.coord_rx}deg )   RY( {self.coord_ry}deg )   RZ( {self.coord_rz}deg )")
        
    def world_eye_image_callback(self, msg):
        """webcam_image 토픽의 메시지를 받아 QLabel에 표시"""
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            
            # OpenCV 이미지를 QImage로 변환
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # QLabel에 QPixmap으로 표시
            self.world_eye_display.setPixmap(QPixmap.fromImage(qt_image))
        except Exception as e:
            self.node.get_logger().error(f"Failed to process image: {e}")

    def hand_eye_image_callback(self, msg):
        """webcam_image 토픽의 메시지를 받아 QLabel에 표시"""
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            
            # OpenCV 이미지를 QImage로 변환
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

            if self.hand_eye_display.pixmap() is None or \
               self.hand_eye_display.pixmap().toImage() != qt_image:
                self.hand_eye_display.setPixmap(QPixmap.fromImage(qt_image))
        except Exception as e:
            self.node.get_logger().error(f"Failed to process image: {e}")
            
    def status_callback(self, msg):
        self.status = msg.data
        self.update_status(self.status)
        
    def coordinate_callback(self, msg):
        self.coordinate = msg.data
        
        # GoalLocation 서비스 요청 객체 생성
        request = GoalLocation.Request()
        request.marker_id = msg.marker_id  # marker_id 추가
        request.x = msg.x
        #request.y = msg.y y값 필요없으므로 생략
        request.z = msg.z
        # Float32 메시지 객체 생성 후 거리 값 할당
        request.distance = Float32()
        request.distance.data = float(msg.distance)  # 거리 값은 float 타입으로 할당
        request.quaternion = msg.quaternion  # 회전 정보 추가

        # ROS2 서비스 호출
        future = self.goal_position.call_async(request)
        future.add_done_callback(self.callback)
        
        #GUI에 좌표, 벡터값 표시
        self.update_coordinate(self.coordinate)
        
    def callback(self, future):
        # 서비스 호출 후 응답 처리
        try:
            response = future.result()
            self.get_logger().info(f'Success: {response.success}, Message: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
    def spin_ros_world(self):
        """ROS2 이벤트 주기적으로 처리"""
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
    def spin_ros_hand(self):
        """ROS2 이벤트 주기적으로 처리"""
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
    def closeEvent(self, event):
        """GUI 종료 시 ROS2 종료"""
        self.node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

# class ConveyorController(Node):
#     def __init__(self):
#         super().__init__('conveyor_controller')
#         self.subscription = self.create_subscription(
#             String,
#             'conveyor_test',
#             self.listener_callback,
#             10
#         )
#         self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 시리얼 포트 설정
#         self.get_logger().info(f"Serial port opened: {self.serial_port.name}")  # Should print the serial port name
#         self.get_logger().info('Conveyor Controller Node Started')
        
#         # 초기 명령 상태를 "Conveyor Stop"으로 설정
#         self.command = "Conveyor Stop"

#         # 0.5초마다 호출되는 타이머 설정
#         self.con_timer = self.create_timer(1.0, self.timer_callback)

#     def listener_callback(self, msg):
#         command = msg.data.strip()  # 토픽에서 문자열 데이터 가져오기 및 공백 제거
#         self.get_logger().info(f"Received command: {command}")
        
#         # 명령이 "Conveyor Start" 또는 "Conveyor Stop"일 때 상태 변경
#         if command == "Conveyor Start":
#             self.command = "Conveyor Start"
#             self.send_serial_command(755)  # Send "START" to Arduino
#         elif command == "Conveyor Stop":
#             self.command = "Conveyor Stop"
#             self.send_serial_command(0)  # Send "STOP" to Arduino
#         else:
#             self.get_logger().warn(f"Unknown command received: {command}")

#     def send_serial_command(self, command):
#         try:
#             command_str = f"{command}\n"  # 명령 뒤에 종료 문자 추가
#             self.serial_port.write(command_str.encode())
#             self.get_logger().info(f"Sent command to Arduino: {command_str.strip()}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to send command: {str(e)}")
            
#     def timer_callback(self):
#         # 1.0초마다 실행되는 콜백
#         self.get_logger().info(f"Current command: {self.command}")
        
#         if self.command == "Conveyor Start":
#             self.send_serial_command(755)  # "START" 명령을 아두이노에 전송
#         elif self.command == "Conveyor Stop":
#             self.send_serial_command(0)  # "STOP" 명령을 아두이노에 전송
#         else:
#             self.get_logger().warn(f"Invalid command: {self.command}")


def main():
    """
    ROS2 Conveyor Node와 PyQt5 GUI를 멀티스레드로 실행하는 통합 main 함수.
    """
    # ROS2 초기화
    rclpy.init()

    # PyQt5 애플리케이션 초기화
    app = QApplication(sys.argv)

    # Conveyor Controller 노드 생성
    #conveyor_controller_node = ConveyorController()

    # MultiThreadedExecutor 생성 및 노드 추가
    executor = MultiThreadedExecutor()
    #executor.add_node(conveyor_controller_node)
    
    # PyQt5 GUI 생성
    gui_node = MainApplication()

    # ROS2 Executor 실행 함수 정의
    def ros_spin():
        try:
            executor.spin()  # ROS2 노드 실행
        except Exception as e:
            print(f"ROS Executor error: {e}")
        finally:
            executor.shutdown()
            rclpy.shutdown()

    # ROS2 Executor를 별도의 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # PyQt5 GUI 실행
    gui_node.show()

    try:
        # PyQt5 이벤트 루프 실행
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 처리
        #conveyor_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
