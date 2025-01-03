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


# ArUco 마커 딕셔너리와 파라미터 설정
aruco_dict_type = cv2.aruco.DICT_5X5_100
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
aruco_params = cv2.aruco.DetectorParameters()

# 카메라 파라미터 로드
camera_matrix = np.load('/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/npy/camera_matrix.npy')
dist_coeffs = np.load('/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/aruco_marker/npy/distortion_coefficients.npy')
                     
# 마커 크기
marker_length = 0.1  # 단위: 미터

# 기준 마커 ID 설정
base_marker_id = 15
comparison_marker_id = 2



class LoginWindow(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.error_label = QLabel()  # 에러 메시지 레이블을 초기화

        # Main Layout
        layout = QVBoxLayout()
        layout.setContentsMargins(100, 50, 100, 50)  # Add margins for spacing
        layout.setSpacing(50)  # Add spacing between widgets

        # Title
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
        self.setWindowTitle("Robot Control System")
        self.setGeometry(100, 100, 1600, 1000)
        self.node = Node("gui_node")  # ROS2 노드 생성
        self.bridge = CvBridge()  # CvBridge 초기화
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
        
        # webcam_image 토픽 Subscribe
        self.subscription_world = self.node.create_subscription(
            Image, 'aruco_detection_image', self.world_eye_image_callback, 10
        )
        
        # hand_image 토픽 Subscribe
        self.subscription_hand = self.node.create_subscription(
            Image, 'hand_image', self.hand_eye_image_callback, 10
        )
        
        # gripper_image 토픽 Subscribe
        # self.subscription_hand = self.node.create_subscription(
        #     Image, 'gripper_image', self.hand_eye_image_callback, 10
        # )
        
        # status 토픽 Subscribe
        self.subscription_status = self.node.create_subscription(
            String, 'status_topic', self.status_callback, 10
        )
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # coordinate 토픽 Subscribe
        self.subscription_coordinate = self.node.create_subscription(
            String, 'marker_relative_position', self.coordinate_callback, qos_profile=qos_profile
        )
        
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
        self.coordinate = msg
        self.coord_x = self.coordinate[0:3]
        self.coord_y = self.coordinate[3:6]
        self.coord_z = self.coordinate[6:9]
        self.coord_rx = self.coordinate[9:12]
        self.coord_ry = self.coordinate[12:15]
        self.coord_rz = self.coordinate[15:18]
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
        self.update_coordinate(self.coordinate)
        
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



class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')

        # ROS 퍼블리셔 설정
        self.marker_publisher = self.create_publisher(
            String, 
            'marker_relative_position', 
            10
        )
        
        self.image_publisher = self.create_publisher(
            Image, 
            'aruco_detection_image', 
            10
        )
        
        self.bridge = CvBridge()  # CvBridge 초기화
        
        # 카메라 입력 설정
        self.cap = cv2.VideoCapture("/dev/video0")  # PC 카메라로부터 영상 가져오기
        if not self.cap.isOpened():
            self.get_logger().error("Unable to open camera!")
            raise RuntimeError("Webcam could not be opened.")

        # ROS 타이머 설정 (30Hz)
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info("ArucoDetection Node Initialized. Publishing to 'marker_relative_position'")

    def process_frame(self):
        """
        카메라 프레임을 읽고 ArUco 마커를 탐지하여 퍼블리시
        """
        ret, frame = self.cap.read()  # 프레임 읽기
        if not ret:
            self.get_logger().error("Failed to read frame from camera")
            return
        
        # OpenCV 이미지를 ROS 메시지로 변환
        # image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # 이미지 퍼블리시
        # self.image_publisher.publish(image_message)

        # ArUco 마커 탐지
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        if ids is not None:
            # ArUco 마커 탐지 로그 출력
            # self.get_logger().info(f"Detected markers: {ids.flatten().tolist()}")

            # 마커의 포즈 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            # 기준 마커 찾기
            base_rvec, base_tvec = None, None
            for i, marker_id in enumerate(ids):
                if marker_id[0] == base_marker_id:
                    base_rvec = rvecs[i]
                    base_tvec = tvecs[i]
                    break

            if base_rvec is not None and base_tvec is not None:
                # ID 14번 마커에 대해 상대 위치 계산
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, base_rvec, base_tvec, 0.05)
                
                for i, marker_id in enumerate(ids):
                    if marker_id[0] == comparison_marker_id:
                        # 현재 마커의 회전 및 이동 벡터
                        marker_rvec = rvecs[i]
                        marker_tvec = tvecs[i]

                        # 기준 마커에 대한 상대 위치 계산
                        relative_rvec, relative_tvec = self.calculate_relative_position(base_rvec, base_tvec, marker_rvec, marker_tvec)
                        x, y, z = relative_tvec.flatten()
                        rx, ry, rz = relative_rvec.flatten()
                        
                        # 정수 거리 값을 계산하여 3자리로 표시
                        x_int = int(abs(x * 1000))
                        y_int = int(abs(y * 1000))
                        z_int = int(abs(z * 1000))

                        # 3자리 숫자로 포맷팅
                        x_formatted = f"{x_int:03}"
                        y_formatted = f"{y_int:03}"
                        z_formatted = f"{z_int:03}"
                        
                        # 각도를 계산하고 3자리로 포맷팅 (rx, ry, rz)
                        rx_angle = int(abs(np.degrees(rx)))  # 라디안 → 도
                        ry_angle = int(abs(np.degrees(ry)))
                        rz_angle = int(abs(np.degrees(rz)))

                        rx_formatted = f"{rx_angle:03}"
                        ry_formatted = f"{ry_angle:03}"
                        rz_formatted = f"{rz_angle:03}"

                        # 18자리 포맷팅 (x, y, z, rx, ry, rz)
                        formatted_output = f"{x_formatted}{y_formatted}{z_formatted}{rx_formatted}{ry_formatted}{rz_formatted}"

                        # 퍼블리시할 메시지 생성
                        # message = (f"Marker ID: {marker_id[0]}, "
                        #            f"Position: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                        #            f"Rotation: rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}")
                        message = formatted_output
                        self.marker_publisher.publish(String(data=message))
                        # self.get_logger().info("[PUBLISHED] " + message)

                        # 마커와 축 표시
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, marker_rvec, marker_tvec, 0.05)

            # 탐지된 마커 그리기
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            self.get_logger().info("No markers detected in this frame.")

        # 결과 영상 출력
        # cv2.imshow("Aruco Detection", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.destroy_node()
        #     rclpy.shutdown()
        #     self.cap.release()
        #     cv2.destroyAllWindows()
        
        # OpenCV 이미지를 ROS 메시지로 변환
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # 이미지 퍼블리시
        self.image_publisher.publish(image_message)

    def calculate_relative_position(self, base_rvec, base_tvec, marker_rvec, marker_tvec):
        """
        기준 마커와 다른 마커 간 상대 위치를 계산
        """
        base_rmat, _ = cv2.Rodrigues(base_rvec)
        marker_rmat, _ = cv2.Rodrigues(marker_rvec)

        relative_rmat = np.dot(np.linalg.inv(base_rmat), marker_rmat)
        relative_rvec, _ = cv2.Rodrigues(relative_rmat)
        relative_tvec = marker_tvec - base_tvec

        return relative_rvec, relative_tvec
    
    def destroy_node(self):
        """노드 종료 시 리소스 정리"""
        self.cap.release()  # 웹캠 리소스 해제
        super().destroy_node()



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
    aruco_marker_node = ArucoDetection()

    # MultiThreadedExecutor 생성 및 노드 추가
    executor = MultiThreadedExecutor()
    #executor.add_node(conveyor_controller_node)
    executor.add_node(aruco_marker_node)
    
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
        aruco_marker_node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
