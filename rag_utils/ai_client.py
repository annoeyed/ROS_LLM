#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 통합 클라이언트
LLM API를 사용하여 AI 기반 응답을 생성하는 통합 인터페이스
"""

import os
import json
import logging
from typing import Dict, Any, List, Optional
from abc import ABC, abstractmethod

class AIClient(ABC):
    """AI 클라이언트 기본 클래스"""
    
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
    
    @abstractmethod
    def generate_response(self, prompt: str, **kwargs) -> str:
        """AI 응답 생성"""
        pass
    
    @abstractmethod
    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        """컨텐츠 분석"""
        pass

class OpenAIClient(AIClient):
    """OpenAI GPT API 클라이언트"""
    
    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        super().__init__()
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        # 환경 변수에서 모델 설정을 우선으로 하고, 기본값은 gpt-4
        self.model = model or os.getenv('OPENAI_MODEL', 'gpt-4')
        
        if not self.api_key:
            self.logger.warning("OpenAI API 키가 설정되지 않았습니다. 환경변수 OPENAI_API_KEY를 설정하세요.")
        
        self.logger.info(f"OpenAI 클라이언트 초기화: 모델={self.model}")
        
        try:
            import openai
            openai.api_key = self.api_key
            self.client = openai
        except ImportError:
            self.logger.error("openai 패키지가 설치되지 않았습니다. pip install openai로 설치하세요.")
            self.client = None
    
    def generate_response(self, prompt: str, **kwargs) -> str:
        """OpenAI GPT로 응답 생성"""
        if not self.client or not self.api_key:
            return self._fallback_response(prompt, "OpenAI API를 사용할 수 없습니다.")
        
        try:
            response = self.client.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "당신은 ROS 보안 전문가입니다. 보안을 고려한 정확하고 실용적인 답변을 제공하세요."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=kwargs.get('max_tokens', 1000),
                temperature=kwargs.get('temperature', 0.3)
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            self.logger.error(f"OpenAI API 호출 실패: {e}")
            return self._fallback_response(prompt, f"API 오류: {str(e)}")
    
    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        """컨텐츠 분석"""
        if analysis_type == "security":
            prompt = f"""
            다음 코드/알고리즘의 보안을 분석하세요:
            
            {content}
            
            다음 형식으로 JSON 응답을 제공하세요:
            {{
                "risk_level": "high/medium/low",
                "risk_score": 0-100,
                "vulnerabilities": [
                    {{
                        "type": "취약점 유형",
                        "description": "설명",
                        "severity": "심각도",
                        "mitigation": "완화 방안"
                    }}
                ],
                "recommendations": ["권장사항1", "권장사항2"]
            }}
            """
        elif analysis_type == "code_generation":
            prompt = f"""
            다음 요구사항에 따라 ROS 2 Python 코드를 생성하세요:
            
            {content}
            
            보안을 고려하여 다음을 포함하세요:
            1. 입력 검증
            2. 에러 처리
            3. 안전한 기본값
            4. 보안 로깅
            """
        else:
            prompt = f"다음 내용을 분석하세요: {content}"
        
        response = self.generate_response(prompt, **kwargs)
        
        # JSON 응답 파싱 시도
        try:
            if analysis_type == "security":
                return json.loads(response)
            else:
                return {"response": response, "type": analysis_type}
        except json.JSONDecodeError:
            return {"response": response, "type": analysis_type, "parsing_error": True}
    
    def _fallback_response(self, prompt: str, error_msg: str) -> str:
        """API 사용 불가 시 대체 응답"""
        return f"AI 분석을 수행할 수 없습니다: {error_msg}\n\n요청된 내용: {prompt}"

class AnthropicClient(AIClient):
    """Anthropic Claude API 클라이언트"""
    
    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        super().__init__()
        self.api_key = api_key or os.getenv('ANTHROPIC_API_KEY')
        # 환경 변수에서 모델 설정을 우선으로 하고, 기본값은 claude-3-sonnet-20240229
        self.model = model or os.getenv('ANTHROPIC_MODEL', 'claude-3-sonnet-20240229')
        
        if not self.api_key:
            self.logger.warning("Anthropic API 키가 설정되지 않았습니다. 환경변수 ANTHROPIC_API_KEY를 설정하세요.")
        
        self.logger.info(f"Anthropic 클라이언트 초기화: 모델={self.model}")
        
        try:
            import anthropic
            self.client = anthropic.Anthropic(api_key=self.api_key)
        except ImportError:
            self.logger.error("anthropic 패키지가 설치되지 않았습니다. pip install anthropic로 설치하세요.")
            self.client = None
    
    def generate_response(self, prompt: str, **kwargs) -> str:
        """Anthropic Claude로 응답 생성"""
        if not self.client or not self.api_key:
            return self._fallback_response(prompt, "Anthropic API를 사용할 수 없습니다.")
        
        try:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=kwargs.get('max_tokens', 1000),
                temperature=kwargs.get('temperature', 0.3),
                messages=[
                    {
                        "role": "user",
                        "content": f"당신은 ROS 보안 전문가입니다. 보안을 고려한 정확하고 실용적인 답변을 제공하세요.\n\n{prompt}"
                    }
                ]
            )
            
            return response.content[0].text
            
        except Exception as e:
            self.logger.error(f"Anthropic API 호출 실패: {e}")
            return self._fallback_response(prompt, f"API 오류: {str(e)}")
    
    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        """컨텐츠 분석"""
        if analysis_type == "security":
            prompt = f"""
            다음 코드/알고리즘의 보안을 분석하세요:
            
            {content}
            
            다음 형식으로 JSON 응답을 제공하세요:
            {{
                "risk_level": "high/medium/low",
                "risk_score": 0-100,
                "vulnerabilities": [
                    {{
                        "type": "취약점 유형",
                        "description": "설명",
                        "severity": "심각도",
                        "mitigation": "완화 방안"
                    }}
                ],
                "recommendations": ["권장사항1", "권장사항2"]
            }}
            """
        elif analysis_type == "code_generation":
            prompt = f"""
            다음 요구사항에 따라 ROS 2 Python 코드를 생성하세요:
            
            {content}
            
            보안을 고려하여 다음을 포함하세요:
            1. 입력 검증
            2. 에러 처리
            3. 안전한 기본값
            4. 보안 로깅
            """
        else:
            prompt = f"다음 내용을 분석하세요: {content}"
        
        response = self.generate_response(prompt, **kwargs)
        
        # JSON 응답 파싱 시도
        try:
            if analysis_type == "security":
                return json.loads(response)
            else:
                return {"response": response, "type": analysis_type}
        except json.JSONDecodeError:
            return {"response": response, "type": analysis_type, "parsing_error": True}
    
    def _fallback_response(self, prompt: str, error_msg: str) -> str:
        """API 사용 불가 시 대체 응답"""
        return f"AI 분석을 수행할 수 없습니다: {error_msg}\n\n요청된 내용: {prompt}"

class MockAIClient(AIClient):
    """테스트용 Mock AI 클라이언트"""
    
    def __init__(self):
        super().__init__()
        self.logger.info("Mock AI 클라이언트가 초기화되었습니다. (테스트용)")
    
    def generate_response(self, prompt: str, **kwargs) -> str:
        """Mock 응답 생성"""
        if "보안" in prompt or "security" in prompt.lower():
            return """
            보안 분석 결과:
            - 위험도: Medium
            - 발견된 이슈: 입력 검증 부족
            - 권장사항: 사용자 입력에 대한 적절한 검증 로직 추가
            """
        elif "코드" in prompt or "code" in prompt.lower():
            return """
            # Mock ROS 2 노드 코드
            import rclpy
            from rclpy.node import Node
            
            class SecureNode(Node):
                def __init__(self):
                    super().__init__('secure_node')
                    # 보안 기능 구현
                    self.get_logger().info('보안 노드가 시작되었습니다.')
            """
        else:
            return f"Mock AI 응답: {prompt[:100]}..."
    
    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        """Mock 컨텐츠 분석"""
        if analysis_type == "security":
            return {
                "risk_level": "medium",
                "risk_score": 65,
                "vulnerabilities": [
                    {
                        "type": "입력 검증 부족",
                        "description": "사용자 입력에 대한 적절한 검증이 없음",
                        "severity": "medium",
                        "mitigation": "입력 검증 로직 추가"
                    }
                ],
                "recommendations": [
                    "입력 검증 강화",
                    "에러 처리 개선",
                    "보안 로깅 구현"
                ]
            }
        elif analysis_type == "code_generation":
            # ROS 2 보안 노드 코드 생성
            return {
                "code": """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Optional
import hashlib
import secrets
import re

class SecureCameraNode(Node):
    \"\"\"보안 카메라 노드 - 인증 및 암호화 기능 포함\"\"\"
    
    def __init__(self) -> None:
        super().__init__('secure_camera_node')
        
        # 보안 설정
        self.api_key = self._load_api_key()
        self.session_token = None
        
        # 퍼블리셔 및 서브스크라이버 설정
        self.publisher = self.create_publisher(String, 'camera_data', 10)
        self.subscription = self.create_subscription(
            String, 'camera_control', self.control_callback, 10
        )
        
        # 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 보안 로깅
        self.secure_log('보안 카메라 노드가 초기화되었습니다', 'info')
        
    def _load_api_key(self) -> str:
        \"\"\"API 키 로드 (환경 변수에서)\"\"\"
        import os
        api_key = os.getenv('CAMERA_API_KEY', 'default_key')
        if api_key == 'default_key':
            self.get_logger().warn('기본 API 키 사용 중 - 프로덕션에서는 환경 변수 설정 필요')
        return api_key
    
    def _authenticate_request(self, credentials: str) -> bool:
        \"\"\"요청 인증\"\"\"
        try:
            # 간단한 인증 로직 (실제로는 더 복잡해야 함)
            if not credentials:
                return False
            
            # API 키 검증
            if credentials == self.api_key:
                return True
            
            # 해시 기반 검증
            expected_hash = hashlib.sha256(self.api_key.encode()).hexdigest()
            if credentials == expected_hash:
                return True
                
            return False
            
        except Exception as e:
            self.secure_log(f'인증 실패: {e}', 'error')
            return False
    
    def _encrypt_data(self, data: str) -> str:
        \"\"\"데이터 암호화\"\"\"
        try:
            # 간단한 해시 기반 암호화 (실제로는 더 강력한 암호화 필요)
            salt = secrets.token_hex(8)
            encrypted = hashlib.sha256((data + salt).encode()).hexdigest()
            return f"{encrypted}:{salt}"
        except Exception as e:
            self.secure_log(f'암호화 실패: {e}', 'error')
            return data
    
    def _decrypt_data(self, encrypted_data: str) -> str:
        \"\"\"데이터 복호화\"\"\"
        try:
            if ':' not in encrypted_data:
                return encrypted_data
            
            encrypted, salt = encrypted_data.split(':', 1)
            # 실제 복호화 로직은 여기에 구현
            return f"decrypted_{encrypted[:10]}"
        except Exception as e:
            self.secure_log(f'복호화 실패: {e}', 'error')
            return encrypted_data
    
    def validate_input(self, input_data: str) -> str:
        \"\"\"입력 데이터 검증\"\"\"
        if not input_data:
            raise ValueError("입력 데이터가 비어있습니다")
        
        # 타입 검증
        if not isinstance(input_data, str):
            raise TypeError("입력 데이터는 문자열이어야 합니다")
        
        # 길이 검증
        if len(input_data) > 1000:
            raise ValueError("입력 데이터가 너무 깁니다")
        
        # 특수 문자 필터링
        if re.search(r'[<>\"']', input_data):
            raise ValueError("허용되지 않는 특수 문자가 포함되어 있습니다")
        
        return input_data.strip()
    
    def secure_log(self, message: str, level: str = 'info') -> None:
        \"\"\"보안 로깅 (민감 정보 마스킹)\"\"\"
        # 민감 정보 마스킹
        masked_message = re.sub(r'password[=:]\\s*\\S+', 'password=***', message)
        masked_message = re.sub(r'api_key[=:]\\s*\\S+', 'api_key=***', message)
        masked_message = re.sub(r'token[=:]\\s*\\S+', 'token=***', message)
        
        if level == 'info':
            self.get_logger().info(masked_message)
        elif level == 'warn':
            self.get_logger().warn(masked_message)
        elif level == 'error':
            self.get_logger().error(masked_message)
        else:
            self.get_logger().info(masked_message)
    
    def control_callback(self, msg: String) -> None:
        \"\"\"카메라 제어 콜백\"\"\"
        try:
            # 입력 검증
            validated_input = self.validate_input(msg.data)
            
            # 인증 확인
            if not self._authenticate_request(validated_input):
                self.secure_log('인증 실패된 제어 요청', 'warn')
                return
            
            # 제어 명령 처리
            if 'start' in validated_input.lower():
                self.secure_log('카메라 시작 명령 수신', 'info')
                # 카메라 시작 로직
            elif 'stop' in validated_input.lower():
                self.secure_log('카메라 정지 명령 수신', 'info')
                # 카메라 정지 로직
            else:
                self.secure_log(f'알 수 없는 제어 명령: {validated_input}', 'warn')
                
        except Exception as e:
            self.secure_log(f'제어 콜백 처리 실패: {e}', 'error')
    
    def timer_callback(self) -> None:
        \"\"\"타이머 콜백 - 카메라 데이터 발행\"\"\"
        try:
            # 카메라 데이터 수집 (시뮬레이션)
            camera_data = f"camera_frame_{self.get_clock().now().nanoseconds}"
            
            # 데이터 암호화
            encrypted_data = self._encrypt_data(camera_data)
            
            # 메시지 발행
            msg = String()
            msg.data = encrypted_data
            self.publisher.publish(msg)
            
            self.secure_log(f'카메라 데이터 발행: {camera_data[:20]}...', 'info')
            
        except Exception as e:
            self.secure_log(f'타이머 콜백 처리 실패: {e}', 'error')\"

def main(args: Optional[list] = None) -> None:
    \"\"\"메인 함수\"\"\"
    try:
        rclpy.init(args=args)
        node = SecureCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if 'node' in locals():
            node.secure_log(f'치명적 오류 발생: {e}', 'error')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()""",
                "metadata": {
                    "description": "ROS 2 기반 보안 카메라 노드 - 인증, 암호화, 입력 검증 기능 포함",
                    "dependencies": ["rclpy", "std_msgs", "hashlib", "secrets"],
                    "usage": "python3 secure_camera_node.py",
                    "security_features": [
                        "입력 검증",
                        "에러 처리", 
                        "보안 로깅",
                        "인증",
                        "암호화",
                        "민감 정보 마스킹"
                    ],
                    "testing_notes": "인증, 암호화, 입력 검증 기능 테스트 필요",
                    "deployment_notes": "환경 변수 CAMERA_API_KEY 설정 필요"
                },
                "security_analysis": {
                    "vulnerabilities": [
                        "기본 API 키 사용 시 보안 위험",
                        "간단한 해시 기반 암호화"
                    ],
                    "mitigations": [
                        "강력한 API 키 사용",
                        "AES 암호화 구현",
                        "JWT 토큰 기반 인증"
                    ],
                    "compliance": [
                        "OWASP ASVS",
                        "ROS 2 보안 가이드라인"
                    ]
                },
                "performance_notes": "타이머 기반 데이터 발행으로 안정적인 성능 제공"
            }
        elif analysis_type == "test_planning":
            return {
                "test_categories": ["기능", "보안", "성능"],
                "test_cases": [
                    {
                        "category": "기능",
                        "name": "기본 동작",
                        "description": "노드 기본 동작 확인"
                    },
                    {
                        "category": "보안", 
                        "name": "인증 검증",
                        "description": "인증 기능 테스트"
                    },
                    {
                        "category": "성능",
                        "name": "응답 시간",
                        "description": "성능 측정"
                    }
                ],
                "test_sequence": ["기능", "보안", "성능"],
                "coverage_goals": ["기본 기능 100%", "보안 검증 90%", "성능 측정 95%"],
                "risk_assessment": "낮음"
            }
        else:
            return {
                "response": f"Mock 분석 결과: {content[:50]}...",
                "type": analysis_type
            }

class AIClientFactory:
    """AI 클라이언트 팩토리"""
    
    @staticmethod
    def create_client(client_type: str = "mock", **kwargs) -> AIClient:
        """AI 클라이언트 생성"""
        if client_type == "openai":
            # OpenAI 모델 설정
            model = kwargs.get('model') or os.getenv('OPENAI_MODEL', 'gpt-4')
            return OpenAIClient(model=model, **kwargs)
        elif client_type == "anthropic":
            # Anthropic 모델 설정
            model = kwargs.get('model') or os.getenv('ANTHROPIC_MODEL', 'claude-3-sonnet-20240229')
            return AnthropicClient(model=model, **kwargs)
        elif client_type == "mock":
            return MockAIClient()
        else:
            logging.warning(f"알 수 없는 클라이언트 타입: {client_type}. Mock 클라이언트를 사용합니다.")
            return MockAIClient()

# 사용 예시
if __name__ == "__main__":
    # Mock 클라이언트 테스트
    client = AIClientFactory.create_client("mock")
    
    # 보안 분석 테스트
    security_result = client.analyze_content(
        "사용자 입력을 직접 사용하는 코드",
        "security"
    )
    print("보안 분석 결과:", json.dumps(security_result, indent=2, ensure_ascii=False))
    
    # 코드 생성 테스트
    code_result = client.generate_response("ROS 2 보안 노드를 생성해주세요")
    print("코드 생성 결과:", code_result)
