#!/usr/bin/env python3
"""
ROS 2 Python Pub/Sub 데모 - 보안 코드 생성 AI용

토픽: /chatter
메시지 타입: std_msgs/msg/String
발행 주기: 1 Hz
QoS: KEEP_LAST depth=10, RELIABLE, VOLATILE
특징: 빈 문자열 금지, 예외 처리, 구조적 로깅
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import logging
import time
from typing import Optional


class TalkerNode(Node):
    """발행자 노드 - 보안 검증된 메시지 발행"""
    
    def __init__(self):
        super().__init__('talker')
        
        # 구조적 로깅 설정
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # QoS 설정 - 요구사항에 맞춤
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(
            String, 
            'chatter', 
            qos_profile
        )
        
        # 타이머 생성 (1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 메시지 카운터
        self.message_count = 0
        
        self.logger.info("Talker 노드가 초기화되었습니다. QoS: RELIABLE, VOLATILE, depth=10")
    
    def validate_message(self, message: str) -> bool:
        """
        메시지 유효성 검증
        
        Args:
            message: 검증할 메시지
            
        Returns:
            bool: 유효한 메시지 여부
        """
        if not message or message.strip() == "":
            self.logger.warning("빈 문자열은 발행할 수 없습니다")
            return False
        
        if len(message) > 1000:  # 메시지 길이 제한
            self.logger.warning("메시지가 너무 깁니다 (최대 1000자)")
            return False
            
        return True
    
    def timer_callback(self):
        """타이머 콜백 - 메시지 발행"""
        try:
            # 메시지 생성
            message = String()
            message.data = f"Hello World {self.message_count}"
            
            # 메시지 유효성 검증
            if not self.validate_message(message.data):
                self.logger.error("메시지 검증 실패")
                return
            
            # 메시지 발행
            self.publisher.publish(message)
            self.message_count += 1
            
            self.logger.info(f"메시지 발행: {message.data}")
            
        except Exception as e:
            self.logger.error(f"메시지 발행 중 오류 발생: {str(e)}")
            # 예외 발생 시 복구 로직
            self.get_logger().error(f"발행 오류: {e}")


class ListenerNode(Node):
    """구독자 노드 - 메시지 수신 및 통계"""
    
    def __init__(self):
        super().__init__('listener')
        
        # 구조적 로깅 설정
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # QoS 설정 - 발행자와 동일
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 구독자 생성
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile
        )
        
        # 통계 변수
        self.message_count = 0
        self.start_time = time.time()
        self.message_times = []
        
        self.logger.info("Listener 노드가 초기화되었습니다")
    
    def listener_callback(self, msg: String):
        """메시지 수신 콜백"""
        try:
            current_time = time.time()
            
            # 메시지 수신 통계
            self.message_count += 1
            self.message_times.append(current_time)
            
            # 로깅
            self.logger.info(f"메시지 수신: {msg.data}")
            
            # 60초 후 통계 출력
            if current_time - self.start_time >= 60.0:
                self.print_statistics()
                
        except Exception as e:
            self.logger.error(f"메시지 수신 처리 중 오류 발생: {str(e)}")
    
    def print_statistics(self):
        """통계 정보 출력"""
        if len(self.message_times) < 2:
            return
            
        # 간격 계산
        intervals = []
        for i in range(1, len(self.message_times)):
            interval = self.message_times[i] - self.message_times[i-1]
            intervals.append(interval)
        
        # 통계 계산
        avg_interval = sum(intervals) / len(intervals)
        sorted_intervals = sorted(intervals)
        p95_index = int(len(sorted_intervals) * 0.95)
        p95_interval = sorted_intervals[p95_index] if p95_index < len(sorted_intervals) else 0
        
        # 결과 출력
        self.logger.info("=== 성능 통계 ===")
        self.logger.info(f"총 수신 메시지: {self.message_count}")
        self.logger.info(f"평균 간격: {avg_interval:.3f}초")
        self.logger.info(f"P95 간격: {p95_interval:.3f}초")
        
        # 성공 기준 검증
        if self.message_count >= 50:
            self.logger.info("✅ 메시지 수신 기준 통과 (≥50개)")
        else:
            self.logger.warning("메시지 수신 기준 미달")
            
        if abs(p95_interval - 1.0) <= 0.1:
            self.logger.info("✅ P95 간격 기준 통과 (≈1.0초)")
        else:
            self.logger.warning("P95 간격 기준 미달")


def main(args=None):
    """메인 함수"""
    try:
        rclpy.init(args=args)
        
        # 노드 생성
        talker = TalkerNode()
        listener = ListenerNode()
        
        # 노드 실행
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(talker)
        executor.add_node(listener)
        
        logging.info("ROS 2 Pub/Sub 데모를 시작합니다...")
        logging.info("60초 후 성능 통계가 출력됩니다.")
        
        executor.spin()
        
    except KeyboardInterrupt:
        logging.info("사용자에 의해 중단되었습니다")
    except Exception as e:
        logging.error(f"예상치 못한 오류 발생: {str(e)}")
    finally:
        # 정리
        if 'talker' in locals():
            talker.destroy_node()
        if 'listener' in locals():
            listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
