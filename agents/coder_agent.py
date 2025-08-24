#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Coder Agent
AI 기반으로 보안을 고려한 ROS 코드를 생성하는 Agent
"""

import sys
import os
import re
from typing import Dict, Any, List, Optional
from .base_agent import BaseAgent, AgentMessage, AgentTask

# 상위 디렉토리 경로 추가 (rag_utils 모듈 접근용)
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class CoderAgent(BaseAgent):
    """AI 기반 ROS 코드 생성 Agent"""
    
    def __init__(self, agent_id: str = "coder_001"):
        super().__init__(agent_id, "Coder Agent")
        
        # AI 클라이언트 초기화
        self.ai_client = None
        
        # 코드 생성 템플릿
        self.code_templates = {
            'basic_node': self._get_basic_node_template(),
            'publisher': self._get_publisher_template(),
            'subscriber': self._get_subscriber_template(),
            'service': self._get_service_template(),
            'action': self._get_action_template(),
            'parameter': self._get_parameter_template()
        }
        
        # 보안 코드 패턴
        self.security_patterns = {
            'input_validation': self._get_input_validation_pattern(),
            'error_handling': self._get_error_handling_pattern(),
            'secure_logging': self._get_secure_logging_pattern(),
            'authentication': self._get_authentication_pattern(),
            'encryption': self._get_encryption_pattern()
        }
    
    def _initialize(self):
        """Coder Agent 초기화"""
        super()._initialize()
        
        try:
            # AI 클라이언트 로드
            self._load_ai_client()
            self.logger.info("AI 클라이언트 로드 완료")
            self.logger.info("AI 기반 ROS 코드 생성 시스템 초기화 완료")
        except Exception as e:
            self.logger.error(f"AI 클라이언트 로드 실패: {e}")
            self.status = 'error'
    
    def _load_ai_client(self):
        """AI 클라이언트 로드"""
        try:
            # 환경변수 로드
            from dotenv import load_dotenv
            import os
            # 현재 파일의 디렉토리를 기준으로 .env 파일 경로 설정
            env_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env')
            load_dotenv(env_path)
            
            from rag_utils.ai_client import AIClientFactory
            
            # 환경변수에서 AI 클라이언트 타입 확인
            ai_client_type = os.getenv('AI_CLIENT_TYPE', 'mock')
            
            # AI 클라이언트 생성
            self.ai_client = AIClientFactory.create_client(ai_client_type)
            
            # AI 클라이언트가 제대로 로드되었는지 확인
            if self.ai_client and hasattr(self.ai_client, 'analyze_content'):
                self.logger.info(f"AI 클라이언트 로드 완료: {ai_client_type}")
                # AI 기능 테스트
                test_response = self.ai_client.analyze_content("테스트", "test")
                if test_response:
                    self.logger.info("AI 기능 테스트 성공")
                else:
                    self.logger.warning("AI 기능 테스트 실패, Mock 클라이언트로 대체")
                    self._fallback_to_mock_client()
            else:
                self.logger.warning("AI 클라이언트 로드 실패, Mock 클라이언트로 대체")
                self._fallback_to_mock_client()
            
        except Exception as e:
            self.logger.error(f"AI 클라이언트 로드 중 오류: {e}")
            self._fallback_to_mock_client()
    
    def _fallback_to_mock_client(self):
        """Mock AI 클라이언트로 대체"""
        try:
            from rag_utils.ai_client import MockAIClient
            self.ai_client = MockAIClient()
            self.logger.info("Mock AI 클라이언트로 대체 완료")
        except Exception as mock_e:
            self.logger.error(f"Mock AI 클라이언트 로드도 실패: {mock_e}")
            self.ai_client = None
    
    def process_message(self, message: AgentMessage) -> AgentMessage:
        """메시지 처리 - 코드 생성 요청 처리"""
        if message.message_type == 'request':
            if 'generate_code' in message.content:
                return self._handle_code_generation_request(message)
            elif 'modify_code' in message.content:
                return self._handle_code_modification_request(message)
            elif 'review_code' in message.content:
                return self._handle_code_review_request(message)
            elif 'optimize_code' in message.content:
                return self._handle_code_optimization_request(message)
            else:
                return self._handle_general_request(message)
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'지원하지 않는 메시지 타입: {message.message_type}'}
            )
    
    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """작업 실행 - AI 기반 코드 생성"""
        self.status = 'busy'
        self.current_task = task
        
        try:
            if task.task_type == 'generate_ros_code':
                result = self._generate_ros_code(task.parameters)
            elif task.task_type == 'modify_code':
                result = self._modify_code(task.parameters)
            elif task.task_type == 'review_code':
                result = self._review_code(task.parameters)
            elif task.task_type == 'optimize_code':
                result = self._optimize_code(task.parameters)
            else:
                result = {'error': f'지원하지 않는 작업 타입: {task.task_type}'}
            
            self.update_task_status(task.task_id, 'completed', result)
            self.status = 'idle'
            return result
            
        except Exception as e:
            error_msg = f'작업 실행 실패: {str(e)}'
            self.update_task_status(task.task_id, 'failed', error=error_msg)
            self.status = 'error'
            return {'error': error_msg}
    
    def _handle_code_generation_request(self, message: AgentMessage) -> AgentMessage:
        """코드 생성 요청 처리"""
        requirements = message.content.get('requirements', '')
        component_type = message.content.get('component_type', 'basic_node')
        security_level = message.content.get('security_level', 'medium')
        
        generated_code = self._generate_ros_code({
            'requirements': requirements,
            'component_type': component_type,
            'security_level': security_level
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'generated_code': generated_code,
                'component_type': component_type,
                'security_level': security_level,
                'status': 'success'
            }
        )
    
    def _handle_code_modification_request(self, message: AgentMessage) -> AgentMessage:
        """코드 수정 요청 처리"""
        existing_code = message.content.get('existing_code', '')
        modification_request = message.content.get('modification_request', '')
        
        modified_code = self._modify_code({
            'existing_code': existing_code,
            'modification_request': modification_request
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'modified_code': modified_code,
                'modification_request': modification_request,
                'status': 'success'
            }
        )
    
    def _handle_code_review_request(self, message: AgentMessage) -> AgentMessage:
        """코드 리뷰 요청 처리"""
        code_snippet = message.content.get('code_snippet', '')
        review_focus = message.content.get('review_focus', 'security')
        
        review_result = self._review_code({
            'code_snippet': code_snippet,
            'review_focus': review_focus
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'review_result': review_result,
                'review_focus': review_focus,
                'status': 'success'
            }
        )
    
    def _handle_code_optimization_request(self, message: AgentMessage) -> AgentMessage:
        """코드 최적화 요청 처리"""
        code_snippet = message.content.get('code_snippet', '')
        optimization_goal = message.content.get('optimization_goal', 'performance')
        
        optimized_code = self._optimize_code({
            'code_snippet': code_snippet,
            'optimization_goal': optimization_goal
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'optimized_code': optimized_code,
                'optimization_goal': optimization_goal,
                'status': 'success'
            }
        )
    
    def _handle_general_request(self, message: AgentMessage) -> AgentMessage:
        """일반 요청 처리"""
        return self.send_message(
            message.sender,
            'response',
            {
                'message': 'Coder Agent가 요청을 처리했습니다.',
                'content': message.content,
                'status': 'success'
            }
        )
    
    def _generate_ros_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI 기반 ROS 코드 생성"""
        requirements = parameters.get('requirements', '')
        component_type = parameters.get('component_type', 'basic_node')
        security_level = parameters.get('security_level', 'medium')
        
        if not requirements:
            return {'error': '요구사항이 제공되지 않았습니다.'}
        
        try:
            if self.ai_client:
                # AI 기반 코드 생성
                ai_generated_code = self._ai_generate_code(requirements, component_type, security_level)
                if ai_generated_code and 'error' not in ai_generated_code:
                    return {
                        'code': ai_generated_code['code'],
                        'metadata': ai_generated_code['metadata'],
                        'security_features': ai_generated_code['security_features'],
                        'ai_enhanced': True,
                        'component_type': component_type,
                        'security_level': security_level
                    }
            
            # AI 실패 시 템플릿 기반 코드 생성
            template_code = self._generate_from_template(requirements, component_type, security_level)
            return {
                'code': template_code['code'],
                'metadata': template_code['metadata'],
                'security_features': template_code['security_features'],
                'ai_enhanced': False,
                'component_type': component_type,
                'security_level': security_level
            }
            
        except Exception as e:
            self.logger.error(f"코드 생성 실패: {e}")
            return {'error': f'코드 생성 실패: {str(e)}'}
    
    def _ai_generate_code(self, requirements: str, component_type: str, security_level: str) -> Dict[str, Any]:
        """AI 기반 코드 생성"""
        try:
            # AI 프롬프트 구성 (더 상세하고 구체적으로)
            ai_prompt = f"""
            다음 요구사항에 따라 고품질의 ROS 2 Python 코드를 생성하세요:
            
            요구사항: {requirements}
            컴포넌트 타입: {component_type}
            보안 수준: {security_level}
            
            다음을 포함하여 완전하고 안전한 Python 코드를 생성하세요:
            
            1. 필요한 import 문 (rclpy, std_msgs, sensor_msgs 등)
            2. 클래스 정의 및 Node 상속
            3. 초기화 메서드 (보안 설정 포함)
            4. 보안 기능:
               - 입력 검증 및 정제
               - 에러 처리 및 복구
               - 보안 로깅 (민감 정보 마스킹)
               - 인증 및 권한 검증
               - 데이터 암호화 (필요시)
            5. 메인 함수 및 예외 처리
            6. 상세한 주석 및 문서화
            7. 타입 힌트 사용
            
            보안 수준별 상세 요구사항:
            - low: 기본 에러 처리, 로깅
            - medium: 입력 검증 + 에러 처리 + 보안 로깅 + 기본 인증
            - high: 모든 보안 기능 + 암호화 + 다중 인증 + 감사 로그
            
            ROS 2 보안 모범 사례:
            - DDS 보안 설정
            - 토픽 암호화
            - 노드 네임스페이스 격리
            - QoS 보안 설정
            
            다음 JSON 형식으로 응답하세요:
            {{
                "code": "완전한 Python 코드 (실행 가능한 상태)",
                "metadata": {{
                    "description": "상세한 코드 설명",
                    "dependencies": ["필요한 패키지 목록"],
                    "usage": "사용법 및 실행 방법",
                    "security_features": ["구현된 보안 기능 목록"],
                    "testing_notes": "테스트 시 고려사항",
                    "deployment_notes": "배포 시 주의사항"
                }},
                "security_analysis": {{
                    "vulnerabilities": ["잠재적 취약점"],
                    "mitigations": ["완화 방안"],
                    "compliance": ["준수하는 보안 표준"]
                }},
                "performance_notes": "성능 최적화 포인트"
            }}
            
            코드는 실제 ROS 2 환경에서 실행 가능해야 하며, 보안 모범 사례를 따라야 합니다.
            """
            
            # AI 코드 생성 수행
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_generation")
            
            if isinstance(ai_response, dict) and 'code' in ai_response:
                # AI 응답 검증 및 개선
                validated_code = self._validate_and_improve_ai_code(ai_response['code'], security_level)
                ai_response['code'] = validated_code
                ai_response['ai_enhanced'] = True
                ai_response['quality_score'] = self._assess_code_quality(validated_code)
                
                self.logger.info(f"AI 기반 코드 생성 성공 (품질 점수: {ai_response['quality_score']})")
                return ai_response
            else:
                # AI 응답 파싱 실패
                self.logger.warning("AI 응답 파싱 실패, 템플릿 기반 생성으로 대체")
                return {'error': 'AI 응답 파싱 실패'}
                
        except Exception as e:
            self.logger.error(f"AI 기반 코드 생성 실패: {e}")
            return {'error': f'AI 코드 생성 실패: {str(e)}'}
    
    def _validate_and_improve_ai_code(self, code: str, security_level: str) -> str:
        """AI 생성 코드 검증 및 개선"""
        try:
            # 기본 검증
            if not code or len(code.strip()) < 100:
                self.logger.warning("AI 생성 코드가 너무 짧음")
                return code
            
            # 보안 수준에 따른 추가 검증
            if security_level == 'high':
                # 고보안 수준 검증
                if 'import hashlib' not in code and 'import cryptography' not in code:
                    code = self._add_encryption_imports(code)
                if 'validate_input' not in code:
                    code = self._add_input_validation(code)
                if 'secure_log' not in code:
                    code = self._add_secure_logging(code)
            
            # 코드 품질 개선
            code = self._improve_code_structure(code)
            code = self._add_error_handling(code)
            code = self._add_type_hints(code)
            
            return code
            
        except Exception as e:
            self.logger.error(f"코드 검증 및 개선 실패: {e}")
            return code
    
    def _assess_code_quality(self, code: str) -> float:
        """코드 품질 평가"""
        try:
            score = 0.0
            max_score = 100.0
            
            # 기본 구조 검사
            if 'class' in code and 'Node' in code:
                score += 20
            if 'def __init__' in code:
                score += 15
            if 'def main' in code:
                score += 15
            
            # 보안 기능 검사
            if 'validate_input' in code:
                score += 10
            if 'try:' in code and 'except' in code:
                score += 10
            if 'get_logger' in code:
                score += 10
            
            # 코드 품질 검사
            if '#' in code:  # 주석
                score += 5
            if '->' in code:  # 타입 힌트
                score += 5
            if 'import' in code:
                score += 5
            if 'rclpy.init' in code and 'rclpy.shutdown' in code:
                score += 5
            
            return min(score, max_score)
            
        except Exception as e:
            self.logger.error(f"코드 품질 평가 실패: {e}")
            return 50.0
    
    def _add_encryption_imports(self, code: str) -> str:
        """암호화 관련 import 추가"""
        try:
            if 'import hashlib' not in code:
                code = "import hashlib\nimport secrets\n" + code
            return code
        except Exception as e:
            self.logger.error(f"암호화 import 추가 실패: {e}")
            return code
    
    def _add_input_validation(self, code: str) -> str:
        """입력 검증 메서드 추가"""
        try:
            validation_method = """
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
        import re
        if re.search(r'[<>\"']', input_data):
            raise ValueError("허용되지 않는 특수 문자가 포함되어 있습니다")
        
        return input_data.strip()
"""
            # 클래스 내부에 메서드 추가
            if 'class' in code and 'def' in code:
                # 마지막 메서드 다음에 추가
                lines = code.split('\n')
                for i, line in enumerate(lines):
                    if line.strip().startswith('def ') and 'def __init__' not in line:
                        # 첫 번째 메서드 다음에 추가
                        lines.insert(i, validation_method)
                        break
                code = '\n'.join(lines)
            
            return code
        except Exception as e:
            self.logger.error(f"입력 검증 메서드 추가 실패: {e}")
            return code
    
    def _add_secure_logging(self, code: str) -> str:
        """보안 로깅 메서드 추가"""
        try:
            logging_method = """
    def secure_log(self, message: str, level: str = 'info') -> None:
        \"\"\"보안 로깅 (민감 정보 마스킹)\"\"\"
        # 민감 정보 마스킹
        import re
        masked_message = re.sub(r'password[=:]\\s*\\S+', 'password=***', message)
        masked_message = re.sub(r'api_key[=:]\\s*\\S+', 'api_key=***', masked_message)
        masked_message = re.sub(r'token[=:]\\s*\\S+', 'token=***', masked_message)
        
        if level == 'info':
            self.get_logger().info(masked_message)
        elif level == 'warn':
            self.get_logger().warn(masked_message)
        elif level == 'error':
            self.get_logger().error(masked_message)
        else:
            self.get_logger().info(masked_message)
"""
            # 클래스 내부에 메서드 추가
            if 'class' in code and 'def' in code:
                lines = code.split('\n')
                for i, line in enumerate(lines):
                    if line.strip().startswith('def ') and 'def __init__' not in line:
                        lines.insert(i, logging_method)
                        break
                code = '\n'.join(lines)
            
            return code
        except Exception as e:
            self.logger.error(f"보안 로깅 메서드 추가 실패: {e}")
            return code
    
    def _improve_code_structure(self, code: str) -> str:
        """코드 구조 개선"""
        try:
            # 기본 구조 검사 및 개선
            if 'if __name__ == "__main__":' not in code:
                code += '\n\nif __name__ == "__main__":\n    main()\n'
            
            return code
        except Exception as e:
            self.logger.error(f"코드 구조 개선 실패: {e}")
            return code
    
    def _add_error_handling(self, code: str) -> str:
        """에러 처리 추가"""
        try:
            # 메인 함수에 에러 처리 추가
            if 'def main(' in code and 'try:' not in code:
                lines = code.split('\n')
                for i, line in enumerate(lines):
                    if 'def main(' in line:
                        # main 함수 시작 부분에 try-except 추가
                        lines.insert(i + 1, '    try:')
                        lines.insert(i + 2, '        rclpy.init(args=args)')
                        lines.insert(i + 3, '        node = GenericNode()')
                        lines.insert(i + 4, '        rclpy.spin(node)')
                        lines.insert(i + 5, '    except KeyboardInterrupt:')
                        lines.insert(i + 6, '        pass')
                        lines.insert(i + 7, '    except Exception as e:')
                        lines.insert(i + 8, '        node.get_logger().error(f"오류 발생: {e}")')
                        lines.insert(i + 9, '    finally:')
                        lines.insert(i + 10, '        if "node" in locals():')
                        lines.insert(i + 11, '            node.destroy_node()')
                        lines.insert(i + 12, '        rclpy.shutdown()')
                        break
                code = '\n'.join(lines)
            
            return code
        except Exception as e:
            self.logger.error(f"에러 처리 추가 실패: {e}")
            return code
    
    def _add_type_hints(self, code: str) -> str:
        """타입 힌트 추가"""
        try:
            # 기본 타입 힌트 추가
            if 'def __init__(self):' in code:
                code = code.replace('def __init__(self):', 'def __init__(self) -> None:')
            
            if 'def timer_callback(self):' in code:
                code = code.replace('def timer_callback(self):', 'def timer_callback(self) -> None:')
            
            return code
        except Exception as e:
            self.logger.error(f"타입 힌트 추가 실패: {e}")
            return code
    
    def _generate_from_template(self, requirements: str, component_type: str, security_level: str) -> Dict[str, Any]:
        """템플릿 기반 코드 생성"""
        try:
            # 기본 템플릿 가져오기
            base_template = self.code_templates.get(component_type, self.code_templates['basic_node'])
            
            # 보안 기능 추가
            security_code = self._add_security_features(base_template, security_level)
            
            # 요구사항에 맞게 커스터마이징
            customized_code = self._customize_code(security_code, requirements)
            
            return {
                'code': customized_code,
                'metadata': {
                    'description': f'{component_type} 기반 코드',
                    'dependencies': ['rclpy', 'std_msgs'],
                    'usage': 'python3 generated_node.py'
                },
                'security_features': self._get_security_features(security_level)
            }
            
        except Exception as e:
            self.logger.error(f"템플릿 기반 코드 생성 실패: {e}")
            return {'error': f'템플릿 코드 생성 실패: {str(e)}'}
    
    def _add_security_features(self, base_code: str, security_level: str) -> str:
        """보안 기능 추가"""
        security_code = base_code
        
        if security_level in ['medium', 'high']:
            # 입력 검증 추가
            security_code += "\n" + self.security_patterns['input_validation']
            
            # 에러 처리 추가
            security_code += "\n" + self.security_patterns['error_handling']
            
            # 보안 로깅 추가
            security_code += "\n" + self.security_patterns['secure_logging']
        
        if security_level == 'high':
            # 인증 추가
            security_code += "\n" + self.security_patterns['authentication']
            
            # 암호화 추가
            security_code += "\n" + self.security_patterns['encryption']
        
        return security_code
    
    def _customize_code(self, base_code: str, requirements: str) -> str:
        """요구사항에 맞게 코드 커스터마이징"""
        # 간단한 텍스트 치환 (실제로는 더 정교한 파싱 필요)
        customized_code = base_code
        
        if 'camera' in requirements.lower():
            customized_code = customized_code.replace('GenericNode', 'CameraNode')
            customized_code = customized_code.replace('generic_topic', 'camera_topic')
        
        if 'authentication' in requirements.lower():
            customized_code = customized_code.replace('# TODO: Add authentication', self.security_patterns['authentication'])
        
        return customized_code
    
    def _get_security_features(self, security_level: str) -> List[str]:
        """보안 수준별 기능 목록"""
        if security_level == 'low':
            return ['기본 에러 처리']
        elif security_level == 'medium':
            return ['입력 검증', '에러 처리', '보안 로깅']
        else:  # high
            return ['입력 검증', '에러 처리', '보안 로깅', '인증', '암호화']
    
    def _modify_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI 기반 코드 수정"""
        existing_code = parameters.get('existing_code', '')
        modification_request = parameters.get('modification_request', '')
        
        if not existing_code or not modification_request:
            return {'error': '기존 코드와 수정 요청이 필요합니다.'}
        
        try:
            if self.ai_client:
                # AI 기반 코드 수정
                ai_modified_code = self._ai_modify_code(existing_code, modification_request)
                if ai_modified_code and 'error' not in ai_modified_code:
                    return {
                        'modified_code': ai_modified_code['code'],
                        'changes': ai_modified_code['changes'],
                        'ai_enhanced': True
                    }
            
            # AI 실패 시 기본 수정
            return {
                'modified_code': existing_code + f"\n# TODO: {modification_request}",
                'changes': [modification_request],
                'ai_enhanced': False
            }
            
        except Exception as e:
            self.logger.error(f"코드 수정 실패: {e}")
            return {'error': f'코드 수정 실패: {str(e)}'}
    
    def _ai_modify_code(self, existing_code: str, modification_request: str) -> Dict[str, Any]:
        """AI 기반 코드 수정"""
        try:
            ai_prompt = f"""
            다음 기존 코드를 수정 요청에 따라 수정하세요:
            
            기존 코드:
            {existing_code}
            
            수정 요청: {modification_request}
            
            수정된 완전한 코드를 제공하고, 변경 사항을 설명하세요.
            
            JSON 형식으로 응답하세요:
            {{
                "code": "수정된 완전한 코드",
                "changes": ["변경 사항 목록"],
                "explanation": "수정 설명"
            }}
            """
            
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_modification")
            
            if isinstance(ai_response, dict) and 'code' in ai_response:
                return ai_response
            else:
                return {'error': 'AI 응답 파싱 실패'}
                
        except Exception as e:
            self.logger.error(f"AI 기반 코드 수정 실패: {e}")
            return {'error': f'AI 코드 수정 실패: {str(e)}'}
    
    def _review_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI 기반 코드 리뷰"""
        code_snippet = parameters.get('code_snippet', '')
        review_focus = parameters.get('review_focus', 'security')
        
        if not code_snippet:
            return {'error': '코드 스니펫이 필요합니다.'}
        
        try:
            if self.ai_client:
                # AI 기반 코드 리뷰
                ai_review = self._ai_review_code(code_snippet, review_focus)
                if ai_review and 'error' not in ai_review:
                    return {
                        'review_result': ai_review,
                        'ai_enhanced': True
                    }
            
            # AI 실패 시 기본 리뷰
            return {
                'review_result': {
                    'score': 7,
                    'issues': ['기본 코드 리뷰만 수행됨'],
                    'recommendations': ['AI 기반 상세 리뷰 권장']
                },
                'ai_enhanced': False
            }
            
        except Exception as e:
            self.logger.error(f"코드 리뷰 실패: {e}")
            return {'error': f'코드 리뷰 실패: {str(e)}'}
    
    def _ai_review_code(self, code_snippet: str, review_focus: str) -> Dict[str, Any]:
        """AI 기반 코드 리뷰"""
        try:
            ai_prompt = f"""
            다음 ROS 2 Python 코드를 {review_focus} 관점에서 리뷰하세요:
            
            코드:
            {code_snippet}
            
            다음 형식으로 JSON 응답을 제공하세요:
            {{
                "score": 0-10,
                "issues": ["발견된 문제점"],
                "recommendations": ["개선 권장사항"],
                "security_concerns": ["보안 우려사항"],
                "best_practices": ["적용된 모범 사례"]
            }}
            """
            
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_review")
            
            if isinstance(ai_response, dict) and 'score' in ai_response:
                return ai_response
            else:
                return {'error': 'AI 응답 파싱 실패'}
                
        except Exception as e:
            self.logger.error(f"AI 기반 코드 리뷰 실패: {e}")
            return {'error': f'AI 코드 리뷰 실패: {str(e)}'}
    
    def _optimize_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI 기반 코드 최적화"""
        code_snippet = parameters.get('code_snippet', '')
        optimization_goal = parameters.get('optimization_goal', 'performance')
        
        if not code_snippet:
            return {'error': '코드 스니펫이 필요합니다.'}
        
        try:
            if self.ai_client:
                # AI 기반 코드 최적화
                ai_optimized = self._ai_optimize_code(code_snippet, optimization_goal)
                if ai_optimized and 'error' not in ai_optimized:
                    return {
                        'optimized_code': ai_optimized['code'],
                        'optimizations': ai_optimized['optimizations'],
                        'ai_enhanced': True
                    }
            
            # AI 실패 시 기본 최적화
            return {
                'optimized_code': code_snippet + f"\n# TODO: {optimization_goal} 최적화 필요",
                'optimizations': [f'{optimization_goal} 최적화'],
                'ai_enhanced': False
            }
            
        except Exception as e:
            self.logger.error(f"코드 최적화 실패: {e}")
            return {'error': f'코드 최적화 실패: {str(e)}'}
    
    def _ai_optimize_code(self, code_snippet: str, optimization_goal: str) -> Dict[str, Any]:
        """AI 기반 코드 최적화"""
        try:
            ai_prompt = f"""
            다음 ROS 2 Python 코드를 {optimization_goal} 관점에서 최적화하세요:
            
            코드:
            {code_snippet}
            
            최적화된 코드를 제공하고, 적용된 최적화 기법을 설명하세요.
            
            JSON 형식으로 응답하세요:
            {{
                "code": "최적화된 코드",
                "optimizations": ["적용된 최적화 기법"],
                "performance_improvements": ["성능 개선 사항"],
                "trade_offs": ["트레이드오프 고려사항"]
            }}
            """
            
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_optimization")
            
            if isinstance(ai_response, dict) and 'code' in ai_response:
                return ai_response
            else:
                return {'error': 'AI 응답 파싱 실패'}
                
        except Exception as e:
            self.logger.error(f"AI 기반 코드 최적화 실패: {e}")
            return {'error': f'AI 코드 최적화 실패: {str(e)}'}
    
    # 코드 템플릿 메서드들
    def _get_basic_node_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GenericNode(Node):
    def __init__(self):
        super().__init__('generic_node')
        self.publisher = self.create_publisher(String, 'generic_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Generic node has been started')
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = GenericNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_publisher_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Publisher node has been started')
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Published message'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_subscriber_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.get_logger().info('Subscriber node has been started')
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_service_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(Trigger, 'trigger_service', self.trigger_callback)
        self.get_logger().info('Service node has been started')
    
    def trigger_callback(self, request, response):
        self.get_logger().info('Service called')
        response.success = True
        response.message = 'Service executed successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_action_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Action node has been started')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # Action execution logic here
        goal_handle.succeed()
        result = Fibonacci.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_parameter_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'default_value')
        self.parameter_value = self.get_parameter('my_parameter').value
        self.get_logger().info(f'Parameter value: {self.parameter_value}')
        self.get_logger().info('Parameter node has been started')
    
    def get_parameter_value(self):
        return self.parameter_value

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    # 보안 패턴 메서드들
    def _get_input_validation_pattern(self) -> str:
        return '''
def validate_input(self, input_data):
    """입력 데이터 검증"""
    if not input_data:
        raise ValueError("입력 데이터가 비어있습니다")
    
    # 타입 검증
    if not isinstance(input_data, str):
        raise TypeError("입력 데이터는 문자열이어야 합니다")
    
    # 길이 검증
    if len(input_data) > 1000:
        raise ValueError("입력 데이터가 너무 깁니다")
    
    # 특수 문자 필터링
    import re
    if re.search(r'[<>"\']', input_data):
        raise ValueError("허용되지 않는 특수 문자가 포함되어 있습니다")
    
    return input_data.strip()'''
    
    def _get_error_handling_pattern(self) -> str:
        return '''
def safe_execute(self, operation, *args, **kwargs):
    """안전한 작업 실행"""
    try:
        result = operation(*args, **kwargs)
        return result
    except Exception as e:
        self.get_logger().error(f"작업 실행 실패: {e}")
        # 기본값 반환 또는 에러 처리
        return None
    
def handle_critical_error(self, error):
    """치명적 오류 처리"""
    self.get_logger().error(f"치명적 오류 발생: {error}")
    # 시스템 안전 상태로 전환
    self.emergency_shutdown()'''
    
    def _get_secure_logging_pattern(self) -> str:
        return '''
def secure_log(self, message, level='info'):
    """보안 로깅"""
    # 민감 정보 마스킹
    import re
    masked_message = re.sub(r'password[=:]\s*\S+', 'password=***', message)
    masked_message = re.sub(r'api_key[=:]\s*\S+', 'api_key=***', masked_message)
    
    if level == 'info':
        self.get_logger().info(masked_message)
    elif level == 'warn':
        self.get_logger().warn(masked_message)
    elif level == 'error':
        self.get_logger().error(masked_message)'''
    
    def _get_authentication_pattern(self) -> str:
        return '''
def authenticate_user(self, credentials):
    """사용자 인증"""
    # TODO: Add authentication
    if not credentials:
        return False
    
    # 인증 로직 구현
    return True
    
def check_permission(self, user, resource):
    """권한 확인"""
    # TODO: Add permission check
    return True'''
    
    def _get_encryption_pattern(self) -> str:
        return '''
def encrypt_data(self, data):
    """데이터 암호화"""
    # TODO: Add encryption
    import hashlib
    return hashlib.sha256(data.encode()).hexdigest()
    
def decrypt_data(self, encrypted_data):
    """데이터 복호화"""
    # TODO: Add decryption
    return encrypted_data'''
    
    def get_status(self) -> Dict[str, Any]:
        """Agent 상태 조회"""
        return {
            'agent_id': self.agent_id,
            'agent_name': self.agent_name,
            'status': self.status,
            'message_queue_length': len(self.message_queue),
            'task_history_length': len(self.task_history),
            'current_task': self.current_task.task_id if self.current_task else None,
            'ai_client': {
                'loaded': self.ai_client is not None,
                'type': self.ai_client.__class__.__name__ if self.ai_client else None,
                'enhanced_generation': self.ai_client is not None
            },
            'code_templates': list(self.code_templates.keys()),
            'security_patterns': list(self.security_patterns.keys())
        }
