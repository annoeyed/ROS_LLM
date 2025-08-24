#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Planner Agent
사용자의 자연어 요청을 분석하여 ROS 코드 생성 계획을 수립하는 Agent
"""

import os
import re
import uuid
from typing import Dict, Any, List, Optional
from .base_agent import BaseAgent, AgentMessage, AgentTask

class PlannerAgent(BaseAgent):
    """사용자 요청을 분석하고 코드 생성 계획을 수립하는 Agent"""
    
    def __init__(self, agent_id: str = "planner_001"):
        super().__init__(agent_id, "Planner Agent")
        
        # ROS 관련 키워드 패턴
        self.ros_patterns = {
            'node': r'\b(node|노드|publisher|subscriber|service|action)\b',
            'topic': r'\b(topic|토픽|message|데이터|통신)\b',
            'sensor': r'\b(sensor|센서|camera|카메라|lidar|라이다|imu|gps)\b',
            'control': r'\b(control|제어|motion|모션|navigation|내비게이션)\b',
            'safety': r'\b(safety|안전|emergency|비상|stop|정지)\b',
            'authentication': r'\b(auth|인증|login|로그인|password|비밀번호)\b',
            'encryption': r'\b(encrypt|암호화|ssl|tls|secure|보안)\b'
        }
        
        # 보안 관련 키워드 패턴
        self.security_patterns = {
            'input_validation': r'\b(validate|검증|input|입력|sanitize|정제)\b',
            'access_control': r'\b(access|접근|permission|권한|role|역할)\b',
            'data_protection': r'\b(protect|보호|privacy|개인정보|encrypt|암호화)\b',
            'logging': r'\b(log|로그|audit|감사|monitor|모니터링)\b'
        }
        
        # 코드 생성 템플릿
        self.code_templates = {
            'basic_node': 'basic_ros_node',
            'publisher_subscriber': 'pub_sub_pattern',
            'service_client': 'service_pattern',
            'action_server': 'action_pattern',
            'parameter_server': 'parameter_pattern',
            'diagnostics': 'diagnostics_pattern'
        }
        
        # AI 클라이언트 초기화
        self.ai_client = None
    
    def _initialize(self):
        """Planner Agent 초기화"""
        super()._initialize()
        
        try:
            # AI 클라이언트 로드
            self._load_ai_client()
            self.logger.info("AI 클라이언트 로드 완료")
            self.logger.info("AI 기반 ROS 코드 생성 계획 수립 시스템 초기화 완료")
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
            
            self.logger.info(f"AI 클라이언트 로드 완료: {ai_client_type}")
            
        except Exception as e:
            self.logger.error(f"AI 클라이언트 로드 중 오류: {e}")
            # AI 클라이언트 로드 실패 시 Mock 클라이언트 사용
            try:
                from rag_utils.ai_client import MockAIClient
                self.ai_client = MockAIClient()
                self.logger.info("Mock AI 클라이언트로 대체")
            except Exception as mock_e:
                self.logger.error(f"Mock AI 클라이언트 로드도 실패: {mock_e}")
                self.ai_client = None
    
    def analyze_request(self, user_request: str) -> Dict[str, Any]:
        """사용자 요청 분석 및 코드 생성 계획 수립 (외부 호출용)"""
        try:
            self.logger.info(f"사용자 요청 분석 시작: {user_request[:100]}...")
            
            # 1. 요청 분석
            analysis_result = self._analyze_user_request(user_request)
            
            # 2. 계획 수립
            planning_result = self._create_code_generation_plan(analysis_result)
            
            # 3. 보안 요구사항 도출
            security_requirements = self._extract_security_requirements(analysis_result)
            
            # 4. 결과 통합
            final_result = {
                'analysis': analysis_result,
                'planning': planning_result,
                'security_requirements': security_requirements,
                'status': 'completed'
            }
            
            self.logger.info("사용자 요청 분석 및 계획 수립 완료")
            return final_result
            
        except Exception as e:
            self.logger.error(f"요청 분석 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e)
            }
    
    def process_message(self, message: AgentMessage) -> AgentMessage:
        """메시지 처리 - 사용자 요청 분석 및 계획 수립"""
        if message.message_type == 'request':
            if 'plan_code_generation' in message.content:
                return self._handle_code_generation_request(message)
            elif 'analyze_requirements' in message.content:
                return self._handle_requirements_analysis(message)
            else:
                return self._handle_general_request(message)
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'지원하지 않는 메시지 타입: {message.message_type}'}
            )
    
    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """작업 실행 - 코드 생성 계획 수립"""
        self.status = 'busy'
        self.current_task = task
        
        try:
            if task.task_type == 'plan_code_generation':
                result = self._plan_code_generation(task.parameters)
            elif task.task_type == 'analyze_security_requirements':
                result = self._analyze_security_requirements(task.parameters)
            elif task.task_type == 'generate_architecture':
                result = self._generate_architecture(task.parameters)
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
        user_request = message.content.get('user_request', '')
        
        # 요청 분석
        analysis = self._analyze_user_request(user_request)
        
        # 계획 수립
        plan = self._create_code_generation_plan(analysis)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'analysis': analysis,
                'plan': plan,
                'status': 'success'
            }
        )
    
    def _handle_requirements_analysis(self, message: AgentMessage) -> AgentMessage:
        """요구사항 분석 요청 처리"""
        requirements = message.content.get('requirements', '')
        
        analysis = self._analyze_requirements(requirements)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'requirements_analysis': analysis,
                'status': 'success'
            }
        )
    
    def _handle_general_request(self, message: AgentMessage) -> AgentMessage:
        """일반 요청 처리"""
        return self.send_message(
            message.sender,
            'response',
            {
                'message': 'Planner Agent가 요청을 받았습니다.',
                'content': message.content,
                'status': 'success'
            }
        )
    
    def _analyze_user_request(self, user_request: str) -> Dict[str, Any]:
        """사용자 요청 분석"""
        try:
            analysis = {
                'request_type': 'unknown',
                'ros_components': [],
                'security_level': 'medium',
                'complexity': 'medium',
                'estimated_lines': 0,
                'dependencies': [],
                'patterns': []
            }
            
            # ROS 컴포넌트 패턴 매칭
            for pattern_name, pattern in self.ros_patterns.items():
                if re.search(pattern, user_request, re.IGNORECASE):
                    analysis['ros_components'].append(pattern_name)
        
            # 보안 레벨 결정
            security_matches = 0
            for pattern_name, pattern in self.security_patterns.items():
                if re.search(pattern, user_request, re.IGNORECASE):
                    security_matches += 1
            
            if security_matches >= 3:
                analysis['security_level'] = 'high'
            elif security_matches >= 1:
                analysis['security_level'] = 'medium'
            else:
                analysis['security_level'] = 'low'
            
            # 복잡도 추정
            if len(analysis['ros_components']) >= 4:
                analysis['complexity'] = 'high'
                analysis['estimated_lines'] = 200
            elif len(analysis['ros_components']) >= 2:
                analysis['complexity'] = 'medium'
                analysis['estimated_lines'] = 100
            else:
                analysis['complexity'] = 'low'
                analysis['estimated_lines'] = 50
            
            # 의존성 추정
            if 'node' in analysis['ros_components']:
                analysis['dependencies'].extend(['rclpy', 'std_msgs'])
            if 'camera' in analysis['ros_components']:
                analysis['dependencies'].extend(['sensor_msgs', 'cv_bridge'])
            if 'control' in analysis['ros_components']:
                analysis['dependencies'].extend(['geometry_msgs', 'nav_msgs'])
        
            return analysis
            
        except Exception as e:
            self.logger.error(f"요청 분석 실패: {e}")
            return {
                'request_type': 'unknown',
                'ros_components': [],
                'security_level': 'medium',
                'complexity': 'medium',
                'estimated_lines': 50,
                'dependencies': ['rclpy', 'std_msgs'],
                'patterns': []
            }
    
    def _create_code_generation_plan(self, analysis: Dict[str, Any]) -> Dict[str, Any]:
        """코드 생성 계획 수립"""
        try:
            plan = {
                'required_agents': ['planner', 'agent', 'security_guide', 'coder', 'simulation'],
                'estimated_time': '30분',
                'phases': [
                    {
                        'phase': 1,
                        'name': '요청 분석 및 계획 수립',
                        'duration': '5분',
                        'description': '사용자 요청 분석 및 코드 생성 계획 수립'
                    },
                    {
                        'phase': 2,
                        'name': '보안 가이드라인 생성',
                        'duration': '5분',
                        'description': 'CWE 기반 보안 가이드라인 및 RAG 검증'
                    },
                    {
                        'phase': 3,
                        'name': '코드 생성',
                        'duration': '10분',
                        'description': '보안 가이드라인을 반영한 ROS 코드 생성'
                    },
                    {
                        'phase': 4,
                        'name': '시뮬레이션 및 검증',
                        'duration': '10분',
                        'description': '코드 시뮬레이션 및 Oracle 검증'
                    }
                ],
                'security_checks': [
                    '입력 검증',
                    '접근 제어',
                    '데이터 보호',
                    '로깅 및 모니터링'
                ],
                'testing_approach': [
                    '단위 테스트',
                    '통합 테스트',
                    '보안 테스트',
                    '성능 테스트'
                ],
                'risk_assessment': 'medium',
                'mitigation_strategies': [
                    '보안 코딩 표준 준수',
                    '정적 분석 도구 사용',
                    '정기적인 보안 검토'
                ]
            }
            
            # 분석 결과에 따른 계획 조정
            if analysis['security_level'] == 'high':
                plan['phases'][1]['duration'] = '8분'
                plan['security_checks'].extend(['암호화', '인증', '권한 관리'])
            
            if analysis['complexity'] == 'high':
                plan['estimated_time'] = '45분'
                plan['phases'][2]['duration'] = '15분'
                plan['phases'][3]['duration'] = '15분'
        
            return plan
            
        except Exception as e:
            self.logger.error(f"계획 수립 실패: {e}")
            return {
                'required_agents': ['planner', 'security_guide', 'coder', 'simulation'],
                'estimated_time': '30분',
                'phases': [],
                'security_checks': [],
                'testing_approach': [],
                'risk_assessment': 'medium',
                'mitigation_strategies': []
            }
    
    def _extract_security_requirements(self, analysis: Dict[str, Any]) -> List[str]:
        """보안 요구사항 도출"""
        requirements = []
        
        # 기본 보안 요구사항
        requirements.extend([
            '입력 데이터 검증',
            '접근 권한 관리',
            '로깅 및 모니터링',
            '에러 처리'
        ])
        
        # 분석 결과에 따른 추가 요구사항
        if analysis['security_level'] == 'high':
            requirements.extend([
                '데이터 암호화',
                '사용자 인증',
                '세션 관리',
                '보안 헤더 설정'
            ])
        
        if 'camera' in analysis['ros_components']:
            requirements.extend([
                '이미지 데이터 보안',
                '개인정보 보호',
                '데이터 전송 보안'
            ])
        
        if 'control' in analysis['ros_components']:
            requirements.extend([
                '제어 명령 검증',
                '안전 장치 연동',
                '비상 정지 기능'
            ])
        
        return requirements
    
    def _analyze_requirements(self, requirements: str) -> Dict[str, Any]:
        """요구사항 분석"""
        return {
            'functional_requirements': self._extract_functional_requirements(requirements),
            'non_functional_requirements': self._extract_non_functional_requirements(requirements),
            'security_requirements': self._extract_security_requirements(requirements),
            'constraints': self._extract_constraints(requirements)
        }
    
    def _extract_functional_requirements(self, requirements: str) -> List[str]:
        """기능 요구사항 추출"""
        functional_keywords = ['must', 'should', 'will', 'shall', '해야', '필요', '구현']
        requirements_list = []
        
        # requirements가 문자열인지 확인
        if not isinstance(requirements, str):
            return requirements_list
        
        sentences = requirements.split('.')
        for sentence in sentences:
            if any(keyword in sentence for keyword in functional_keywords):
                requirements_list.append(sentence.strip())
        
        return requirements_list
    
    def _extract_non_functional_requirements(self, requirements: str) -> List[str]:
        """비기능 요구사항 추출"""
        non_functional_keywords = ['performance', '성능', 'reliability', '신뢰성', 'scalability', '확장성']
        requirements_list = []
        
        # requirements가 문자열인지 확인
        if not isinstance(requirements, str):
            return requirements_list
        
        sentences = requirements.split('.')
        for sentence in sentences:
            if any(keyword in sentence for keyword in non_functional_keywords):
                requirements_list.append(sentence.strip())
        
        return requirements_list
    
    def _extract_security_requirements(self, requirements: str) -> List[str]:
        """보안 요구사항 추출"""
        security_keywords = ['security', '보안', 'authentication', '인증', 'authorization', '권한', 'encryption', '암호화']
        requirements_list = []
        
        # requirements가 문자열인지 확인
        if not isinstance(requirements, str):
            return requirements_list
        
        sentences = requirements.split('.')
        for sentence in sentences:
            if any(keyword in sentence for keyword in security_keywords):
                requirements_list.append(sentence.strip())
        
        return requirements_list
    
    def _extract_constraints(self, requirements: str) -> List[str]:
        """제약사항 추출"""
        constraint_keywords = ['constraint', '제약', 'limit', '제한', 'must not', '하지 않아야']
        constraints_list = []
        
        # requirements가 문자열인지 확인
        if not isinstance(requirements, str):
            return constraints_list
        
        sentences = requirements.split('.')
        for sentence in sentences:
            if any(keyword in sentence for keyword in constraint_keywords):
                constraints_list.append(sentence.strip())
        
        return constraints_list
    
    def _analyze_security_requirements(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """보안 요구사항 분석"""
        # 보안 요구사항 분석 로직 구현
        return {'security_analysis': 'completed'}
    
    def _generate_architecture(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """아키텍처 생성"""
        # 아키텍처 생성 로직 구현
        return {'architecture': 'generated'}
    
    def _ai_analyze_user_request(self, user_request: str) -> Dict[str, Any]:
        """AI 기반 사용자 요청 분석"""
        if not self.ai_client:
            return self._analyze_user_request(user_request)
        
        try:
            # AI 분석 수행
            ai_response = self.ai_client.analyze_content(user_request, "planning")
            
            # AI 응답이 딕셔너리이고 필요한 키를 포함하는지 확인
            if isinstance(ai_response, dict) and 'ros_components' in ai_response:
                # AI 응답 파싱 성공 - 기본 분석과 동일한 형식으로 변환
                return {
                    'request_type': 'ai_analyzed',
                    'ros_components': ai_response.get('ros_components', []),
                    'security_level': 'medium',  # 기본값
                    'complexity': 'medium',      # 기본값
                    'estimated_lines': 100,     # 기본값
                    'dependencies': ['rclpy', 'std_msgs'],  # 기본값
                    'patterns': []
                }
            else:
                # AI 응답 파싱 실패 시 기본 분석 사용
                return self._analyze_user_request(user_request)
                
        except Exception as e:
            self.logger.error(f"AI 기반 요청 분석 실패: {e}")
            return self._analyze_user_request(user_request)
    
    def _ai_create_code_generation_plan(self, analysis: Dict[str, Any], user_request: str) -> Dict[str, Any]:
        """AI 기반 코드 생성 계획 수립"""
        if not self.ai_client:
            return self._create_code_generation_plan(analysis)
        
        try:
            # AI 프롬프트 구성
            ai_prompt = f"""
            다음 분석 결과를 바탕으로 ROS 2 코드 생성 계획을 수립하세요:
            
            분석 결과: {analysis}
            원본 요청: {user_request}
            
            다음 형식으로 JSON 응답을 제공하세요:
            {{
                "required_agents": ["필요한 Agent 목록"],
                "estimated_time": "예상 소요 시간",
                "phases": [
                    {{
                        "phase": 1,
                        "name": "단계명",
                        "duration": "소요시간",
                        "description": "상세 설명"
                    }}
                ],
                "security_checks": ["보안 검사 항목"],
                "testing_approach": ["테스트 접근법"],
                "risk_assessment": "위험도 평가",
                "mitigation_strategies": ["위험 완화 전략"]
            }}
            """
            
            # AI 계획 수립 수행
            ai_response = self.ai_client.analyze_content(ai_prompt, "planning")
            
            if isinstance(ai_response, dict) and 'required_agents' in ai_response:
                # AI 응답 파싱 성공
                return ai_response
            else:
                # AI 응답 파싱 실패 시 기본 계획 사용
                return self._create_code_generation_plan(analysis)
                
        except Exception as e:
            self.logger.error(f"AI 기반 계획 수립 실패: {e}")
            return self._create_code_generation_plan(analysis)
    
    def plan_ros_code_generation(self, user_request: str) -> Dict[str, Any]:
        """AI 기반 ROS 코드 생성 계획 수립 (외부 호출용)"""
        try:
            # AI 기반 요청 분석
            analysis = self._ai_analyze_user_request(user_request)
            
            # AI 기반 계획 수립
            plan = self._ai_create_code_generation_plan(analysis, user_request)
            
            return {
                'analysis': analysis,
                'plan': plan,
                'status': 'success',
                'ai_enhanced': self.ai_client is not None
            }
            
        except Exception as e:
            self.logger.error(f"AI 기반 계획 수립 실패: {e}")
            # AI 실패 시 기본 분석 사용
            try:
                analysis = self._analyze_user_request(user_request)
                plan = self._create_code_generation_plan(analysis)
                return {
                    'analysis': analysis,
                    'plan': plan,
                    'status': 'success',
                    'ai_enhanced': False,
                    'fallback': True
                }
            except Exception as fallback_e:
                return {
                    'error': f'계획 수립 실패: {str(e)} (fallback도 실패: {str(fallback_e)})',
                    'status': 'error'
                }
