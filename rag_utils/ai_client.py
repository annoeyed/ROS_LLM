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
    
    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-4"):
        super().__init__()
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        self.model = model
        
        if not self.api_key:
            self.logger.warning("OpenAI API 키가 설정되지 않았습니다. 환경변수 OPENAI_API_KEY를 설정하세요.")
        
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
    
    def __init__(self, api_key: Optional[str] = None, model: str = "claude-3-sonnet-20240229"):
        super().__init__()
        self.api_key = api_key or os.getenv('ANTHROPIC_API_KEY')
        self.model = model
        
        if not self.api_key:
            self.logger.warning("Anthropic API 키가 설정되지 않았습니다. 환경변수 ANTHROPIC_API_KEY를 설정하세요.")
        
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
            return OpenAIClient(**kwargs)
        elif client_type == "anthropic":
            return AnthropicClient(**kwargs)
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
