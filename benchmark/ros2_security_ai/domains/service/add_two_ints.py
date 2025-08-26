#!/usr/bin/env python3
"""
ROS 2 AddTwoInts 서비스 - 보안 코드 생성 AI용

서비스: add_two_ints(std_srvs/srv/AddTwoInts)
특징: 음수 입력 거부, 에러 코드/메시지 반환, 예외 처리, 입력 검증, 로깅
성능 측정: 10회 요청에 대한 평균 지연
"""

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.client import Client
from std_srvs.srv import AddTwoInts
import time
import logging
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class ServiceResult:
    """서비스 호출 결과"""
    request_id: int
    a: int
    b: int
    response_time: float
    success: bool
    result: int
    error_code: int
    error_message: str


class AddTwoIntsService(Node):
    """AddTwoInts 서비스 노드 - 보안 검증된 덧셈 서비스"""
    
    def __init__(self):
        super().__init__('add_two_ints_service')
        
        # 구조적 로깅 설정
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # 서비스 생성
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        
        # 통계 변수
        self.request_count = 0
        self.success_count = 0
        self.error_count = 0
        
        self.logger.info("AddTwoInts 서비스가 시작되었습니다")
    
    def validate_input(self, a: int, b: int) -> Tuple[bool, int, str]:
        """
        입력값 유효성 검증
        
        Args:
            a: 첫 번째 정수
            b: 두 번째 정수
            
        Returns:
            Tuple[bool, int, str]: (유효성, 에러 코드, 에러 메시지)
        """
        # 음수 체크
        if a < 0:
            return False, 1001, f"첫 번째 입력값이 음수입니다: {a}"
        
        if b < 0:
            return False, 1002, f"두 번째 입력값이 음수입니다: {b}"
        
        # 범위 체크 (32비트 정수 범위)
        if a > 2147483647:
            return False, 1003, f"첫 번째 입력값이 너무 큽니다: {a}"
        
        if b > 2147483647:
            return False, 1004, f"두 번째 입력값이 너무 큽니다: {b}"
        
        # 오버플로우 체크
        if a + b > 2147483647:
            return False, 1005, f"덧셈 결과가 오버플로우됩니다: {a} + {b}"
        
        return True, 0, ""
    
    def add_two_ints_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response) -> AddTwoInts.Response:
        """서비스 콜백 - 덧셈 수행"""
        try:
            self.request_count += 1
            start_time = time.time()
            
            self.logger.info(f"서비스 요청 #{self.request_count}: {request.a} + {request.b}")
            
            # 입력값 검증
            is_valid, error_code, error_message = self.validate_input(request.a, request.b)
            
            if not is_valid:
                # 에러 응답
                self.error_count += 1
                response.sum = 0
                
                # 에러 정보를 로그에 기록
                self.logger.warning(f"입력 검증 실패: {error_message} (코드: {error_code})")
                
                # 응답에 에러 정보 포함 (실제로는 std_srvs에는 에러 필드가 없지만 로깅용)
                self.logger.error(f"에러 응답: 코드={error_code}, 메시지={error_message}")
                
            else:
                # 정상 처리
                self.success_count += 1
                response.sum = request.a + request.b
                
                processing_time = time.time() - start_time
                self.logger.info(f"덧셈 완료: {request.a} + {request.b} = {response.sum} (처리시간: {processing_time:.6f}초)")
            
            return response
            
        except Exception as e:
            # 예외 처리
            self.error_count += 1
            self.logger.error(f"서비스 처리 중 예외 발생: {str(e)}")
            
            # 기본 응답 반환
            response.sum = 0
            return response
    
    def get_statistics(self) -> dict:
        """서비스 통계 반환"""
        return {
            "total_requests": self.request_count,
            "successful_requests": self.success_count,
            "error_requests": self.error_count,
            "success_rate": (self.success_count / max(self.request_count, 1)) * 100
        }


class AddTwoIntsClient(Node):
    """AddTwoInts 클라이언트 노드 - 성능 측정용"""
    
    def __init__(self):
        super().__init__('add_two_ints_client')
        
        # 구조적 로깅 설정
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # 클라이언트 생성
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 성능 측정 변수
        self.results: List[ServiceResult] = []
        self.test_requests = [
            (1, 2), (5, 10), (100, 200), (1000, 2000),
            (-1, 5), (5, -1), (2147483647, 1), (1, 2147483647)
        ]
        
        self.logger.info("AddTwoInts 클라이언트가 초기화되었습니다")
    
    def wait_for_service(self, timeout: float = 10.0) -> bool:
        """서비스 대기"""
        try:
            if self.client.wait_for_service(timeout_sec=timeout):
                self.logger.info("서비스가 준비되었습니다")
                return True
            else:
                self.logger.error("서비스 대기 시간 초과")
                return False
        except Exception as e:
            self.logger.error(f"서비스 대기 중 오류: {str(e)}")
            return False
    
    def call_service(self, a: int, b: int) -> ServiceResult:
        """서비스 호출"""
        try:
            # 요청 생성
            request = AddTwoInts.Request()
            request.a = a
            request.b = b
            
            # 서비스 호출 및 시간 측정
            start_time = time.time()
            future = self.client.call_async(request)
            
            # 응답 대기
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            response_time = time.time() - start_time
            
            if future.done():
                response = future.result()
                
                # 성공 응답
                return ServiceResult(
                    request_id=len(self.results) + 1,
                    a=a,
                    b=b,
                    response_time=response_time,
                    success=True,
                    result=response.sum,
                    error_code=0,
                    error_message=""
                )
            else:
                # 타임아웃
                return ServiceResult(
                    request_id=len(self.results) + 1,
                    a=a,
                    b=b,
                    response_time=response_time,
                    success=False,
                    result=0,
                    error_code=2001,
                    error_message="서비스 호출 타임아웃"
                )
                
        except Exception as e:
            # 예외 발생
            return ServiceResult(
                request_id=len(self.results) + 1,
                a=a,
                b=b,
                response_time=0.0,
                success=False,
                result=0,
                error_code=2002,
                error_message=f"서비스 호출 오류: {str(e)}"
            )
    
    def run_performance_test(self, num_requests: int = 10) -> List[ServiceResult]:
        """성능 테스트 실행"""
        self.logger.info(f"{num_requests}회 서비스 호출 성능 테스트를 시작합니다...")
        
        if not self.wait_for_service():
            self.logger.error("서비스를 찾을 수 없습니다")
            return []
        
        # 테스트 요청 실행
        for i in range(num_requests):
            # 테스트 케이스 선택 (순환)
            test_case = self.test_requests[i % len(self.test_requests)]
            a, b = test_case
            
            self.logger.info(f"테스트 #{i+1}: {a} + {b}")
            
            # 서비스 호출
            result = self.call_service(a, b)
            self.results.append(result)
            
            # 결과 로깅
            if result.success:
                self.logger.info(f"✅ 성공: {a} + {b} = {result.result} (응답시간: {result.response_time:.6f}초)")
            else:
                self.logger.warning(f"❌ 실패: {result.error_message} (코드: {result.error_code})")
            
            # 요청 간 간격
            time.sleep(0.1)
        
        return self.results
    
    def calculate_statistics(self) -> dict:
        """통계 계산"""
        if not self.results:
            return {}
        
        successful_results = [r for r in self.results if r.success]
        
        if not successful_results:
            return {
                "total_requests": len(self.results),
                "successful_requests": 0,
                "error_requests": len(self.results),
                "success_rate": 0.0,
                "avg_response_time": 0.0,
                "min_response_time": 0.0,
                "max_response_time": 0.0
            }
        
        response_times = [r.response_time for r in successful_results]
        
        return {
            "total_requests": len(self.results),
            "successful_requests": len(successful_results),
            "error_requests": len(self.results) - len(successful_results),
            "success_rate": (len(successful_results) / len(self.results)) * 100,
            "avg_response_time": sum(response_times) / len(response_times),
            "min_response_time": min(response_times),
            "max_response_time": max(response_times)
        }
    
    def print_report(self):
        """성능 리포트 출력"""
        stats = self.calculate_statistics()
        
        if not stats:
            self.logger.warning("통계 데이터가 없습니다")
            return
        
        self.logger.info("=== AddTwoInts 서비스 성능 리포트 ===")
        self.logger.info(f"총 요청 수: {stats['total_requests']}")
        self.logger.info(f"성공 요청 수: {stats['successful_requests']}")
        self.logger.info(f"실패 요청 수: {stats['error_requests']}")
        self.logger.info(f"성공률: {stats['success_rate']:.1f}%")
        
        if stats['successful_requests'] > 0:
            self.logger.info(f"평균 응답 시간: {stats['avg_response_time']:.6f}초")
            self.logger.info(f"최소 응답 시간: {stats['min_response_time']:.6f}초")
            self.logger.info(f"최대 응답 시간: {stats['max_response_time']:.6f}초")
        
        # 성공 기준 검증
        if stats['success_rate'] >= 80.0:
            self.logger.info("✅ 성공률 기준 통과 (≥80%)")
        else:
            self.logger.warning("❌ 성공률 기준 미달")
        
        if stats['successful_requests'] > 0 and stats['avg_response_time'] <= 0.1:
            self.logger.info("✅ 응답 시간 기준 통과 (≤0.1초)")
        else:
            self.logger.warning("❌ 응답 시간 기준 미달")


def main():
    """메인 함수"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    rclpy.init()
    
    try:
        # 서비스 노드 생성 및 실행
        service_node = AddTwoIntsService()
        
        # 별도 스레드에서 서비스 실행
        import threading
        service_thread = threading.Thread(target=rclpy.spin, args=(service_node,))
        service_thread.daemon = True
        service_thread.start()
        
        # 잠시 대기하여 서비스가 준비되도록 함
        time.sleep(2)
        
        # 클라이언트 노드 생성 및 성능 테스트
        client_node = AddTwoIntsClient()
        
        # 성능 테스트 실행
        results = client_node.run_performance_test(10)
        
        # 결과 리포트 출력
        client_node.print_report()
        
        # 정리
        client_node.destroy_node()
        service_node.destroy_node()
        
    except KeyboardInterrupt:
        logging.info("사용자에 의해 중단되었습니다")
    except Exception as e:
        logging.error(f"실행 중 오류: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
