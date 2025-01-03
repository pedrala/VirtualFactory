import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sqlite3
from rclpy.qos import QoSProfile, ReliabilityPolicy



class StatusPublisher(Node):
    def __init__(self):
        super().__init__('rotating_status_publisher')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(String, 'status_topic', qos_profile=qos_profile)
        
        # 데이터베이스 경로 및 상태 순환용 변수
        self.db_path = "/home/viator/ws/aruco/aruco_grasp_conveyor/src/aruco_grasp_conveyor/aruco_grasp_conveyor/database/working_status.db"
        self.current_index = 1  # 시작 id
        
        # 타이머 생성 (1.0초 간격으로 실행)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info("RotatingStatusPublisher node initialized.")

    def fetch_status(self, index):
        """데이터베이스에서 특정 id의 status 값을 가져옵니다."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # id에 해당하는 status 값 가져오기
            cursor.execute("SELECT status FROM orders WHERE id = ?", (index,))
            row = cursor.fetchone()
            conn.close()
            
            return row[0] if row else None
        except Exception as e:
            self.get_logger().error(f"Database error: {e}")
            return None

    def get_next_index(self):
        """다음 id를 가져옵니다. (순환 방식)"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # 현재 테이블에서 최대 id 값을 가져옵니다.
            cursor.execute("SELECT MAX(id) FROM orders")
            max_id = cursor.fetchone()[0]
            conn.close()
            
            # 순환 로직: 현재 id를 증가시키고 최대 id를 초과하면 다시 1로
            self.current_index = 1 if self.current_index >= max_id else self.current_index + 1
        except Exception as e:
            self.get_logger().error(f"Database error: {e}")
            self.current_index = 1

    def publish_status(self):
        """현재 id의 status 값을 퍼블리시합니다."""
        status = self.fetch_status(self.current_index)
        if status:
            msg = String()
            msg.data = status
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published status: {status}")
        else:
            self.get_logger().warn(f"No status value fetched for id: {self.current_index}")
        
        # 다음 id로 이동
        self.get_next_index()



def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
