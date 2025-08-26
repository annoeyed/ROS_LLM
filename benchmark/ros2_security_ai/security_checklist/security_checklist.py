#!/usr/bin/env python3
"""
보안 체크리스트 - ROS 2 보안 AI용

보안 가이드가 생성할 검증 규칙(Oracles용)을 구현합니다.
"""

import json
import logging
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass
from enum import Enum


class ValidationLevel(Enum):
    """검증 수준"""
    CRITICAL = "CRITICAL"      # 치명적 오류
    HIGH = "HIGH"              # 높은 위험
    MEDIUM = "MEDIUM"          # 중간 위험
    LOW = "LOW"                # 낮은 위험
    INFO = "INFO"              # 정보


class ValidationResult:
    """검증 결과"""
    
    def __init__(self, rule_name: str, level: ValidationLevel, passed: bool, 
                 message: str, details: Dict[str, Any] = None):
        self.rule_name = rule_name
        self.level = level
        self.passed = passed
        self.message = message
        self.details = details or {}
    
    def to_dict(self) -> Dict[str, Any]:
        """딕셔너리로 변환"""
        return {
            "rule_name": self.rule_name,
            "level": self.level.value,
            "passed": self.passed,
            "message": self.message,
            "details": self.details
        }
    
    def __str__(self) -> str:
        status = "✅ 통과" if self.passed else "❌ 실패"
        return f"[{self.level.value}] {self.rule_name}: {status} - {self.message}"


class SecurityChecklist:
    """보안 체크리스트 검증기"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # 검증 규칙 정의
        self.validation_rules = self._define_validation_rules()
        
        # 검증 결과 저장
        self.validation_results: List[ValidationResult] = []
    
    def _define_validation_rules(self) -> Dict[str, Dict[str, Any]]:
        """검증 규칙 정의"""
        return {
            "input_validation": {
                "name": "입력 검증",
                "description": "빈 문자열·음수 금지, 범위 체크, 파라미터 기본값/타입 체크",
                "level": ValidationLevel.CRITICAL,
                "category": "보안"
            },
            "exception_handling": {
                "name": "예외 처리/로깅",
                "description": "모든 통신/전환 경로에 구조적 로깅, 예외 핸들러",
                "level": ValidationLevel.HIGH,
                "category": "안정성"
            },
            "qos_compliance": {
                "name": "QoS 준수",
                "description": "요청된 QoS(RELIABLE/KEEP_LAST/depth=10) 준수 여부",
                "level": ValidationLevel.MEDIUM,
                "category": "성능"
            },
            "lifecycle_management": {
                "name": "라이프사이클 관리",
                "description": "비활성 상태 발행 금지, 전환 실패 시 복구",
                "level": ValidationLevel.HIGH,
                "category": "안정성"
            },
            "security_sros2": {
                "name": "보안(SROS2)",
                "description": "올바른 enclave/permissions일 때만 통신, 무권한 차단 로그 확인",
                "level": ValidationLevel.CRITICAL,
                "category": "보안"
            }
        }
    
    def validate_input_validation(self, code_content: str, config: Dict[str, Any]) -> ValidationResult:
        """입력 검증 규칙 검사"""
        try:
            # 빈 문자열 체크
            empty_string_check = self._check_empty_string_validation(code_content)
            
            # 음수 체크
            negative_check = self._check_negative_validation(code_content)
            
            # 범위 체크
            range_check = self._check_range_validation(code_content)
            
            # 타입 체크
            type_check = self._check_type_validation(code_content)
            
            # 종합 결과
            all_passed = all([empty_string_check, negative_check, range_check, type_check])
            
            details = {
                "empty_string_validation": empty_string_check,
                "negative_validation": negative_check,
                "range_validation": range_check,
                "type_validation": type_check
            }
            
            if all_passed:
                message = "모든 입력 검증 규칙이 준수되었습니다"
            else:
                message = "일부 입력 검증 규칙이 누락되었습니다"
            
            return ValidationResult(
                rule_name="입력 검증",
                level=ValidationLevel.CRITICAL,
                passed=all_passed,
                message=message,
                details=details
            )
            
        except Exception as e:
            return ValidationResult(
                rule_name="입력 검증",
                level=ValidationLevel.CRITICAL,
                passed=False,
                message=f"입력 검증 검사 중 오류 발생: {str(e)}"
            )
    
    def _check_empty_string_validation(self, code_content: str) -> bool:
        """빈 문자열 검증 체크"""
        # 빈 문자열 체크 패턴들
        patterns = [
            "if not message", "if not msg", "if not data",
            "message.strip()", "msg.strip()", "data.strip()",
            "len(message) > 0", "len(msg) > 0", "len(data) > 0",
            "message != \"\"", "msg != \"\"", "data != \"\""
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_negative_validation(self, code_content: str) -> bool:
        """음수 검증 체크"""
        # 음수 체크 패턴들
        patterns = [
            "if value < 0", "if val < 0", "if num < 0",
            "value >= 0", "val >= 0", "num >= 0",
            "abs(value)", "abs(val)", "abs(num)"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_range_validation(self, code_content: str) -> bool:
        """범위 검증 체크"""
        # 범위 체크 패턴들
        patterns = [
            "if value > max_value", "if val > max_val",
            "if value < min_value", "if val < min_val",
            "value <= max_value", "val <= max_val",
            "value >= min_value", "val >= min_val"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_type_validation(self, code_content: str) -> bool:
        """타입 검증 체크"""
        # 타입 체크 패턴들
        patterns = [
            "isinstance(", "type(", "type()",
            "if isinstance(", "if type("
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def validate_exception_handling(self, code_content: str, config: Dict[str, Any]) -> ValidationResult:
        """예외 처리/로깅 규칙 검사"""
        try:
            # try-except 블록 체크
            try_except_check = "try:" in code_content and "except" in code_content
            
            # 로깅 체크
            logging_check = self._check_logging_implementation(code_content)
            
            # 예외 핸들러 체크
            exception_handler_check = self._check_exception_handlers(code_content)
            
            # 종합 결과
            all_passed = all([try_except_check, logging_check, exception_handler_check])
            
            details = {
                "try_except_blocks": try_except_check,
                "logging_implementation": logging_check,
                "exception_handlers": exception_handler_check
            }
            
            if all_passed:
                message = "모든 예외 처리/로깅 규칙이 준수되었습니다"
            else:
                message = "일부 예외 처리/로깅 규칙이 누락되었습니다"
            
            return ValidationResult(
                rule_name="예외 처리/로깅",
                level=ValidationLevel.HIGH,
                passed=all_passed,
                message=message,
                details=details
            )
            
        except Exception as e:
            return ValidationResult(
                rule_name="예외 처리/로깅",
                level=ValidationLevel.HIGH,
                passed=False,
                message=f"예외 처리/로깅 검사 중 오류 발생: {str(e)}"
            )
    
    def _check_logging_implementation(self, code_content: str) -> bool:
        """로깅 구현 체크"""
        # 로깅 패턴들
        patterns = [
            "logging.", "self.logger.", "self.get_logger()",
            "log.info", "log.warning", "log.error", "log.debug"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_exception_handlers(self, code_content: str) -> bool:
        """예외 핸들러 체크"""
        # 예외 핸들러 패턴들
        patterns = [
            "except Exception", "except ValueError", "except TypeError",
            "except RuntimeError", "except OSError"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def validate_qos_compliance(self, code_content: str, config: Dict[str, Any]) -> ValidationResult:
        """QoS 준수 규칙 검사"""
        try:
            # QoS 설정 체크
            qos_profile_check = "QoSProfile" in code_content
            
            # RELIABLE 체크
            reliable_check = "ReliabilityPolicy.RELIABLE" in code_content
            
            # KEEP_LAST 체크
            keep_last_check = "HistoryPolicy.KEEP_LAST" in code_content
            
            # depth=10 체크
            depth_check = "depth=10" in code_content or "depth = 10" in code_content
            
            # VOLATILE 체크
            volatile_check = "DurabilityPolicy.VOLATILE" in code_content
            
            # 종합 결과
            all_passed = all([qos_profile_check, reliable_check, keep_last_check, depth_check, volatile_check])
            
            details = {
                "qos_profile": qos_profile_check,
                "reliable": reliable_check,
                "keep_last": keep_last_check,
                "depth_10": depth_check,
                "volatile": volatile_check
            }
            
            if all_passed:
                message = "모든 QoS 요구사항이 준수되었습니다"
            else:
                message = "일부 QoS 요구사항이 누락되었습니다"
            
            return ValidationResult(
                rule_name="QoS 준수",
                level=ValidationLevel.MEDIUM,
                passed=all_passed,
                message=message,
                details=details
            )
            
        except Exception as e:
            return ValidationResult(
                rule_name="QoS 준수",
                level=ValidationLevel.MEDIUM,
                passed=False,
                message=f"QoS 준수 검사 중 오류 발생: {str(e)}"
            )
    
    def validate_lifecycle_management(self, code_content: str, config: Dict[str, Any]) -> ValidationResult:
        """라이프사이클 관리 규칙 검사"""
        try:
            # 라이프사이클 노드 체크
            lifecycle_node_check = "LifecycleNode" in code_content
            
            # 상태 체크
            state_check = self._check_lifecycle_state_management(code_content)
            
            # 전환 실패 처리 체크
            transition_failure_check = self._check_transition_failure_handling(code_content)
            
            # 비활성 상태 발행 금지 체크
            inactive_publish_check = self._check_inactive_publish_prevention(code_content)
            
            # 종합 결과
            all_passed = all([lifecycle_node_check, state_check, transition_failure_check, inactive_publish_check])
            
            details = {
                "lifecycle_node": lifecycle_node_check,
                "state_management": state_check,
                "transition_failure_handling": transition_failure_check,
                "inactive_publish_prevention": inactive_publish_check
            }
            
            if all_passed:
                message = "모든 라이프사이클 관리 규칙이 준수되었습니다"
            else:
                message = "일부 라이프사이클 관리 규칙이 누락되었습니다"
            
            return ValidationResult(
                rule_name="라이프사이클 관리",
                level=ValidationLevel.HIGH,
                passed=all_passed,
                message=message,
                details=details
            )
            
        except Exception as e:
            return ValidationResult(
                rule_name="라이프사이클 관리",
                level=ValidationLevel.HIGH,
                passed=False,
                message=f"라이프사이클 관리 검사 중 오류 발생: {str(e)}"
            )
    
    def _check_lifecycle_state_management(self, code_content: str) -> bool:
        """라이프사이클 상태 관리 체크"""
        patterns = [
            "on_configure", "on_activate", "on_deactivate", "on_cleanup",
            "get_current_state", "get_available_states"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_transition_failure_handling(self, code_content: str) -> bool:
        """전환 실패 처리 체크"""
        patterns = [
            "try:", "except", "transition_failed", "transition_error"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_inactive_publish_prevention(self, code_content: str) -> bool:
        """비활성 상태 발행 금지 체크"""
        patterns = [
            "if self.is_active", "if self.get_current_state", "active_state"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def validate_security_sros2(self, code_content: str, config: Dict[str, Any]) -> ValidationResult:
        """보안(SROS2) 규칙 검사"""
        try:
            # SROS2 관련 체크
            sros2_check = self._check_sros2_implementation(code_content)
            
            # 권한 체크
            permissions_check = self._check_permissions_validation(code_content)
            
            # 접근 제어 체크
            access_control_check = self._check_access_control(code_content)
            
            # 암호화 체크
            encryption_check = self._check_encryption_implementation(code_content)
            
            # 종합 결과
            all_passed = all([sros2_check, permissions_check, access_control_check, encryption_check])
            
            details = {
                "sros2_implementation": sros2_check,
                "permissions_validation": permissions_check,
                "access_control": access_control_check,
                "encryption": encryption_check
            }
            
            if all_passed:
                message = "모든 보안(SROS2) 규칙이 준수되었습니다"
            else:
                message = "일부 보안(SROS2) 규칙이 누락되었습니다"
            
            return ValidationResult(
                rule_name="보안(SROS2)",
                level=ValidationLevel.CRITICAL,
                passed=all_passed,
                message=message,
                details=details
            )
            
        except Exception as e:
            return ValidationResult(
                rule_name="보안(SROS2)",
                level=ValidationLevel.CRITICAL,
                passed=False,
                message=f"보안(SROS2) 검사 중 오류 발생: {str(e)}"
            )
    
    def _check_sros2_implementation(self, code_content: str) -> bool:
        """SROS2 구현 체크"""
        patterns = [
            "ros2 security", "keystore", "governance", "permissions",
            "ROS_SECURITY_KEYSTORE", "ROS_SECURITY_STRATEGY"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_permissions_validation(self, code_content: str) -> bool:
        """권한 검증 체크"""
        patterns = [
            "permissions.xml", "grant", "subject_name", "validity"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_access_control(self, code_content: str) -> bool:
        """접근 제어 체크"""
        patterns = [
            "access_control", "access_denied", "insufficient_permissions",
            "unauthorized_access", "permission_check"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def _check_encryption_implementation(self, code_content: str) -> bool:
        """암호화 구현 체크"""
        patterns = [
            "encryption", "crypto", "ssl", "tls", "certificate"
        ]
        
        return any(pattern in code_content for pattern in patterns)
    
    def run_all_validations(self, code_content: str, config: Dict[str, Any] = None) -> List[ValidationResult]:
        """모든 검증 규칙 실행"""
        self.logger.info("보안 체크리스트 검증을 시작합니다...")
        
        config = config or {}
        
        # 각 검증 규칙 실행
        validations = [
            self.validate_input_validation(code_content, config),
            self.validate_exception_handling(code_content, config),
            self.validate_qos_compliance(code_content, config),
            self.validate_lifecycle_management(code_content, config),
            self.validate_security_sros2(code_content, config)
        ]
        
        self.validation_results = validations
        
        # 결과 요약
        total_rules = len(validations)
        passed_rules = sum(1 for v in validations if v.passed)
        critical_failures = sum(1 for v in validations if not v.passed and v.level == ValidationLevel.CRITICAL)
        
        self.logger.info(f"검증 완료: {passed_rules}/{total_rules} 규칙 통과")
        if critical_failures > 0:
            self.logger.warning(f"⚠️ 치명적 오류: {critical_failures}개")
        
        return validations
    
    def generate_report(self) -> str:
        """검증 리포트 생성"""
        if not self.validation_results:
            return "검증 결과가 없습니다."
        
        report = "=== 보안 체크리스트 검증 리포트 ===\n\n"
        
        # 결과 요약
        total_rules = len(self.validation_results)
        passed_rules = sum(1 for v in self.validation_results if v.passed)
        critical_failures = sum(1 for v in self.validation_results if not v.passed and v.level == ValidationLevel.CRITICAL)
        high_failures = sum(1 for v in self.validation_results if not v.passed and v.level == ValidationLevel.HIGH)
        
        report += f"총 검증 규칙: {total_rules}개\n"
        report += f"통과 규칙: {passed_rules}개\n"
        report += f"실패 규칙: {total_rules - passed_rules}개\n"
        report += f"치명적 오류: {critical_failures}개\n"
        report += f"높은 위험: {high_failures}개\n\n"
        
        # 각 규칙별 결과
        for result in self.validation_results:
            report += str(result) + "\n"
            if result.details:
                for key, value in result.details.items():
                    status = "✅" if value else "❌"
                    report += f"  - {key}: {status}\n"
            report += "\n"
        
        # 전체 평가
        if critical_failures == 0 and high_failures == 0:
            report += "🎉 모든 중요 규칙이 통과되었습니다!"
        elif critical_failures == 0:
            report += "⚠️ 치명적 오류는 없지만 높은 위험 요소가 있습니다."
        else:
            report += "🚨 치명적 보안 오류가 발견되었습니다!"
        
        return report
    
    def save_results(self, filename: str = "security_checklist_results.json"):
        """검증 결과를 JSON 파일로 저장"""
        try:
            import time
            data = {
                "timestamp": time.time(),
                "total_rules": len(self.validation_results),
                "passed_rules": sum(1 for v in self.validation_results if v.passed),
                "failed_rules": sum(1 for v in self.validation_results if not v.passed),
                "critical_failures": sum(1 for v in self.validation_results if not v.passed and v.level == ValidationLevel.CRITICAL),
                "results": [result.to_dict() for result in self.validation_results]
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            self.logger.info(f"검증 결과가 {filename}에 저장되었습니다")
            
        except Exception as e:
            self.logger.error(f"결과 저장 오류: {str(e)}")


def main():
    """메인 함수"""
    import time
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 예시 코드 (실제로는 파일에서 읽거나 사용자 입력)
    example_code = """
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import logging

class SecureNode(Node):
    def __init__(self):
        super().__init__('secure_node')
        
        # 로깅 설정
        self.logger = logging.getLogger(__name__)
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(String, 'chatter', qos_profile)
        
    def publish_message(self, message):
        try:
            # 입력 검증
            if not message or message.strip() == "":
                self.logger.warning("빈 문자열은 발행할 수 없습니다")
                return False
            
            if len(message) > 1000:
                self.logger.warning("메시지가 너무 깁니다")
                return False
            
            # 메시지 발행
            msg = String()
            msg.data = message
            self.publisher.publish(msg)
            
            self.logger.info(f"메시지 발행: {message}")
            return True
            
        except Exception as e:
            self.logger.error(f"메시지 발행 중 오류: {str(e)}")
            return False
"""
    
    # 보안 체크리스트 실행
    checklist = SecurityChecklist()
    
    try:
        # 모든 검증 실행
        results = checklist.run_all_validations(example_code)
        
        # 리포트 생성 및 출력
        report = checklist.generate_report()
        print(report)
        
        # 결과 저장
        checklist.save_results()
        
    except Exception as e:
        logging.error(f"보안 체크리스트 실행 중 오류: {str(e)}")


if __name__ == '__main__':
    main()
