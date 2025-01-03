import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import ssl



class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber')
        
        # 구독자 생성
        self.subscription = self.create_subscription(
            String,
            'status_topic',
            self.status_callback,
            10
        )
        self.get_logger().info("StatusSubscriber node initialized.")

    def status_callback(self, msg):
        """수신된 메시지를 확인하여 'error' 상태일 경우 이메일을 보냄."""
        status = msg.data
        self.get_logger().info(f"Received status: {status}")
        
        if status == 'error':
            self.send_email("Error detected", "An error has been detected in the system!")

    def send_email(self, subject, body):
        """이메일 발송 함수"""
        try:
            sender_email = "rokey01338@gmail.com"  # Gmail 이메일 주소
            receiver_email = "jsh10198@naver.com"  # 받는 사람 이메일
            password = "dyzh vhkl vboj nzoz"  # Gmail 앱 비밀번호

            # 이메일 메시지 구성
            msg = MIMEMultipart()
            msg['From'] = sender_email
            msg['To'] = receiver_email
            msg['Subject'] = subject

            msg.attach(MIMEText(body, 'plain'))

            # SSL/TLS 연결 및 이메일 발송
            context = ssl.create_default_context()
            with smtplib.SMTP_SSL('smtp.gmail.com', 465, context=context) as server:
                server.login(sender_email, password)  # Gmail 이메일과 비밀번호로 로그인
                server.sendmail(sender_email, receiver_email, msg.as_string())
                
            self.get_logger().info("Email sent successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to send email: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = StatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
