#!/usr/bin/env python3
"""
Apex.AI performance_test 벤치마크 - ROS 2 보안 AI용

퍼블리셔 1, 서브스크라이버 1
메시지 타입: String
메시지 크기: {32B, 1KB, 64KB}
발행 주기: 1 Hz
기간: 60초
QoS: RELIABLE, KEEP_LAST, depth=10, VOLATILE
"""

import subprocess
import json
import time
import logging
from typing import Dict, List, Tuple
from dataclasses import dataclass


@dataclass
class BenchmarkResult:
    """벤치마크 결과 데이터 클래스"""
    message_size: str
    p50_latency: float
    p95_latency: float
    loss_rate: float
    throughput: float
    status: str


class ApexBenchmark:
    """Apex.AI performance_test 벤치마크 실행기"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # 벤치마크 설정
        self.message_sizes = ["32B", "1KB", "64KB"]
        self.duration = 60  # 초
        self.frequency = 1  # Hz
        self.qos_depth = 10
        
        # 결과 저장
        self.results: List[BenchmarkResult] = []
    
    def check_performance_test_installed(self) -> bool:
        """performance_test가 설치되어 있는지 확인"""
        try:
            result = subprocess.run(
                ["ros2", "run", "performance_test", "--help"],
                capture_output=True,
                text=True,
                timeout=10
            )
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
    
    def install_performance_test(self) -> bool:
        """performance_test 설치"""
        try:
            self.logger.info("performance_test를 설치합니다...")
            
            # Ubuntu/Debian 기반
            install_cmd = [
                "sudo", "apt", "update", "&&",
                "sudo", "apt", "install", "-y", "ros-humble-performance-test"
            ]
            
            result = subprocess.run(
                " ".join(install_cmd),
                shell=True,
                capture_output=True,
                text=True,
                timeout=300
            )
            
            if result.returncode == 0:
                self.logger.info("performance_test 설치 완료")
                return True
            else:
                self.logger.error(f"설치 실패: {result.stderr}")
                return False
                
        except Exception as e:
            self.logger.error(f"설치 중 오류: {str(e)}")
            return False
    
    def run_single_benchmark(self, message_size: str) -> BenchmarkResult:
        """단일 메시지 크기에 대한 벤치마크 실행"""
        try:
            self.logger.info(f"{message_size} 메시지 크기로 벤치마크를 시작합니다...")
            
            # performance_test 명령어 구성
            cmd = [
                "ros2", "run", "performance_test", "performance_test",
                "--pub", "1",                    # 퍼블리셔 1개
                "--sub", "1",                    # 서브스크라이버 1개
                "--msg", "String",               # String 메시지
                "--size", message_size,          # 메시지 크기
                "--rate", str(self.frequency),   # 1 Hz
                "--duration", str(self.duration), # 60초
                "--qos", "reliable",             # RELIABLE
                "--qos-depth", str(self.qos_depth), # depth=10
                "--qos-durability", "volatile"   # VOLATILE
            ]
            
            # 벤치마크 실행
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.duration + 30  # 여유 시간
            )
            
            # 결과 파싱
            return self.parse_performance_output(result.stdout, message_size)
            
        except subprocess.TimeoutExpired:
            self.logger.error(f"{message_size} 벤치마크 시간 초과")
            return BenchmarkResult(
                message_size=message_size,
                p50_latency=0.0,
                p95_latency=0.0,
                loss_rate=100.0,
                throughput=0.0,
                status="TIMEOUT"
            )
        except Exception as e:
            self.logger.error(f"{message_size} 벤치마크 오류: {str(e)}")
            return BenchmarkResult(
                message_size=message_size,
                p50_latency=0.0,
                p95_latency=0.0,
                loss_rate=100.0,
                throughput=0.0,
                status="ERROR"
            )
    
    def parse_performance_output(self, output: str, message_size: str) -> BenchmarkResult:
        """performance_test 출력 파싱"""
        try:
            # 기본값 설정
            p50_latency = 0.0
            p95_latency = 0.0
            loss_rate = 0.0
            throughput = 0.0
            status = "SUCCESS"
            
            # 출력에서 수치 추출 (실제 출력 형식에 따라 조정 필요)
            lines = output.split('\n')
            for line in lines:
                if "P50 latency" in line:
                    try:
                        p50_latency = float(line.split()[-1])
                    except ValueError:
                        pass
                elif "P95 latency" in line:
                    try:
                        p95_latency = float(line.split()[-1])
                    except ValueError:
                        pass
                elif "Loss rate" in line:
                    try:
                        loss_rate = float(line.split()[-1].rstrip('%'))
                    except ValueError:
                        pass
                elif "Throughput" in line:
                    try:
                        throughput = float(line.split()[-1])
                    except ValueError:
                        pass
            
            return BenchmarkResult(
                message_size=message_size,
                p50_latency=p50_latency,
                p95_latency=p95_latency,
                loss_rate=loss_rate,
                throughput=throughput,
                status=status
            )
            
        except Exception as e:
            self.logger.error(f"출력 파싱 오류: {str(e)}")
            return BenchmarkResult(
                message_size=message_size,
                p50_latency=0.0,
                p95_latency=0.0,
                loss_rate=100.0,
                throughput=0.0,
                status="PARSE_ERROR"
            )
    
    def run_all_benchmarks(self) -> List[BenchmarkResult]:
        """모든 메시지 크기에 대한 벤치마크 실행"""
        self.logger.info("Apex.AI performance_test 벤치마크를 시작합니다...")
        
        # performance_test 설치 확인
        if not self.check_performance_test_installed():
            self.logger.warning("performance_test가 설치되지 않았습니다. 설치를 시도합니다...")
            if not self.install_performance_test():
                self.logger.error("performance_test 설치에 실패했습니다")
                return []
        
        # 각 메시지 크기별 벤치마크 실행
        for message_size in self.message_sizes:
            result = self.run_single_benchmark(message_size)
            self.results.append(result)
            
            # 결과 출력
            self.logger.info(f"=== {message_size} 결과 ===")
            self.logger.info(f"P50 지연: {result.p50_latency:.3f}ms")
            self.logger.info(f"P95 지연: {result.p95_latency:.3f}ms")
            self.logger.info(f"손실률: {result.loss_rate:.3f}%")
            self.logger.info(f"처리량: {result.throughput:.2f} msg/s")
            self.logger.info(f"상태: {result.status}")
            
            # 성능 기준 검증
            self.verify_performance_criteria(result)
        
        return self.results
    
    def verify_performance_criteria(self, result: BenchmarkResult):
        """성능 기준 검증"""
        if result.status != "SUCCESS":
            self.logger.warning(f"❌ {result.message_size}: 벤치마크 실패")
            return
        
        # 성능 기준: p95 ≤ 50ms, 손실률 ≤ 0.1%
        if result.p95_latency <= 50.0:
            self.logger.info(f"✅ {result.message_size}: P95 지연 기준 통과 (≤50ms)")
        else:
            self.logger.warning(f"❌ {result.message_size}: P95 지연 기준 미달 ({result.p95_latency:.3f}ms)")
        
        if result.loss_rate <= 0.1:
            self.logger.info(f"✅ {result.message_size}: 손실률 기준 통과 (≤0.1%)")
        else:
            self.logger.warning(f"❌ {result.message_size}: 손실률 기준 미달 ({result.loss_rate:.3f}%)")
    
    def generate_report(self) -> str:
        """벤치마크 리포트 생성"""
        if not self.results:
            return "벤치마크 결과가 없습니다."
        
        report = "=== Apex.AI Performance Test 벤치마크 리포트 ===\n\n"
        report += "설정:\n"
        report += f"- 퍼블리셔: 1개\n"
        report += f"- 서브스크라이버: 1개\n"
        report += f"- 메시지 타입: String\n"
        report += f"- 발행 주기: {self.frequency} Hz\n"
        report += f"- 기간: {self.duration}초\n"
        report += f"- QoS: RELIABLE, KEEP_LAST, depth={self.qos_depth}, VOLATILE\n\n"
        
        report += "결과:\n"
        report += f"{'크기':<8} {'P50(ms)':<10} {'P95(ms)':<10} {'손실률(%)':<12} {'처리량':<10} {'상태':<10}\n"
        report += "-" * 70 + "\n"
        
        for result in self.results:
            report += f"{result.message_size:<8} {result.p50_latency:<10.3f} {result.p95_latency:<10.3f} "
            report += f"{result.loss_rate:<12.3f} {result.throughput:<10.2f} {result.status:<10}\n"
        
        # 성공률 계산
        successful = sum(1 for r in self.results if r.status == "SUCCESS")
        success_rate = (successful / len(self.results)) * 100
        
        report += f"\n성공률: {success_rate:.1f}% ({successful}/{len(self.results)})\n"
        
        return report
    
    def save_results(self, filename: str = "apex_benchmark_results.json"):
        """결과를 JSON 파일로 저장"""
        try:
            data = {
                "timestamp": time.time(),
                "settings": {
                    "publishers": 1,
                    "subscribers": 1,
                    "message_type": "String",
                    "frequency": self.frequency,
                    "duration": self.duration,
                    "qos_depth": self.qos_depth
                },
                "results": [
                    {
                        "message_size": r.message_size,
                        "p50_latency": r.p50_latency,
                        "p95_latency": r.p95_latency,
                        "loss_rate": r.loss_rate,
                        "throughput": r.throughput,
                        "status": r.status
                    }
                    for r in self.results
                ]
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            self.logger.info(f"결과가 {filename}에 저장되었습니다")
            
        except Exception as e:
            self.logger.error(f"결과 저장 오류: {str(e)}")


def main():
    """메인 함수"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    benchmark = ApexBenchmark()
    
    try:
        # 벤치마크 실행
        results = benchmark.run_all_benchmarks()
        
        # 리포트 생성 및 출력
        report = benchmark.generate_report()
        print(report)
        
        # 결과 저장
        benchmark.save_results()
        
    except KeyboardInterrupt:
        logging.info("사용자에 의해 중단되었습니다")
    except Exception as e:
        logging.error(f"벤치마크 실행 중 오류: {str(e)}")


if __name__ == '__main__':
    main()
