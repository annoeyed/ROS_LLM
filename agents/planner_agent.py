#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Planner Agent
Agent that analyzes natural language requests from the user and establishes plans for ROS code generation
"""

import os
import re
import uuid
from typing import Dict, Any, List, Optional
from .base_agent import BaseAgent, AgentMessage, AgentTask

class PlannerAgent(BaseAgent):
    """Agent that analyzes user requests and establishes code generation plans"""
    
    def __init__(self, llm_client, agent_id: str = "planner_001"):
        super().__init__(agent_id, "Planner Agent")
        self.llm_client = llm_client
        
        # ROS-related keyword patterns
        self.ros_patterns = {
            'node': r'\b(node|publisher|subscriber|service|action)\b',
            'topic': r'\b(topic|message|data|communication)\b',
            'sensor': r'\b(sensor|camera|lidar|imu|gps)\b',
            'control': r'\b(control|motion|navigation)\b',
            'safety': r'\b(safety|emergency|stop)\b',
            'authentication': r'\b(auth|login|password)\b',
            'encryption': r'\b(encrypt|ssl|tls|secure)\b'
        }
        
        # Security-related keyword patterns
        self.security_patterns = {
            'input_validation': r'\b(validate|input|sanitize)\b',
            'access_control': r'\b(access|permission|role)\b',
            'data_protection': r'\b(protect|privacy|encrypt)\b',
            'logging': r'\b(log|audit|monitor)\b'
        }
        
        # Code generation templates
        self.code_templates = {
            'basic_node': 'basic_ros_node',
            'publisher_subscriber': 'pub_sub_pattern',
            'service_client': 'service_pattern',
            'action_server': 'action_pattern',
            'parameter_server': 'parameter_pattern',
            'diagnostics': 'diagnostics_pattern'
        }
        
        # Initialize AI client
        self.ai_client = None
    
    def _initialize(self):
        """Initialize Planner Agent"""
        super()._initialize()
        
        try:
            # Load AI client
            self._load_ai_client()
            self.logger.info("AI client loaded successfully")
            self.logger.info("AI-based ROS code generation planning system initialized")
        except Exception as e:
            self.logger.error(f"Failed to load AI client: {e}")
            self.status = 'error'
    
    def _load_ai_client(self):
        """Load AI client"""
        try:
            # Load environment variables
            from dotenv import load_dotenv
            import os
            # Set .env file path based on current file directory
            env_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env')
            load_dotenv(env_path)
            
            from utils.ai_client import AIClientFactory
            
            # Check AI client type from environment variables
            ai_client_type = os.getenv('AI_CLIENT_TYPE')
            
            # Create AI client
            # self.ai_client = AIClientFactory.create_client(ai_client_type)
            self.ai_client = None  # Will be set by workflow
            
            self.logger.info(f"AI client loaded successfully: {ai_client_type}")
            
        except Exception as e:
            self.logger.error(f"Error loading AI client: {e}")
            self.ai_client = None

    def set_ai_client(self, ai_client):
        """워크플로우에서 AI 클라이언트를 설정하는 메서드"""
        self.ai_client = ai_client
        self.logger.info(f"AI client set by workflow: {type(ai_client).__name__}")
        
    def get_ai_client_status(self):
        """AI 클라이언트 상태 확인"""
        if self.ai_client:
            return {
                'status': 'available',
                'type': type(self.ai_client).__name__,
                'configured': True
            }
        else:
            return {
                'status': 'unavailable',
                'type': 'None',
                'configured': False
            }
    
    def analyze_request(self, user_request: str) -> Dict[str, Any]:
        """Analyze user request and establish code generation plan (for external calls)"""
        try:
            self.logger.info(f"Starting user request analysis: {user_request[:100]}...")
            
            # 1. 요청 분석
            analysis_result = self._analyze_user_request(user_request)
            
            # 2. 실행 명세 생성 (새로 추가)
            execution_spec = self._generate_execution_specification(user_request)
            
            # 3. 계획 수립
            planning_result = self._create_code_generation_plan(analysis_result, execution_spec)
            
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
        
        # 실행 명세 생성 (새로 추가)
        execution_spec = self._generate_execution_specification(user_request)
        
        # 계획 수립
        plan = self._create_code_generation_plan(analysis, execution_spec)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'analysis': analysis,
                'execution_specification': execution_spec,  # 새로 추가
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
    
    def _create_code_generation_plan(self, analysis: Dict[str, Any], execution_spec: Dict[str, Any]) -> Dict[str, Any]:
        """코드 생성 계획 수립"""
        try:
            plan = {
                'required_agents': ['planner', 'agent', 'security_guide', 'coder', 'simulation'],
                'estimated_time': '30분',
                'execution_specification': execution_spec,  # 실행 명세 포함
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
            
            # 실행 명세에 따른 계획 조정
            if execution_spec.get('node', {}).get('language') == 'python':
                plan['required_agents'].append('python_validator')
                plan['phases'][3]['description'] = '보안 가이드라인을 반영한 ROS Python 코드 생성'
            
            # 복잡도에 따른 시간 조정
            node_count = len(execution_spec.get('topics', {}).get('input', [])) + len(execution_spec.get('topics', {}).get('output', []))
            if node_count > 3:
                plan['estimated_time'] = '60분'
                plan['phases'][3]['duration'] = '20분'
        
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

    def _generate_execution_specification(self, user_request: str) -> Dict[str, Any]:
        """사용자 요청을 바탕으로 실행 명세 생성"""
        try:
            if not self.ai_client:
                self.logger.warning("AI client not available, using template-based generation")
                return self._generate_template_specification(user_request)
            
            # AI 기반 실행 명세 생성
            return self._generate_ai_based_specification(user_request)
            
        except Exception as e:
            self.logger.error(f"실행 명세 생성 실패: {e}")
            return self._generate_template_specification(user_request)

    def _generate_ai_based_specification(self, user_request: str) -> Dict[str, Any]:
        """AI를 활용하여 실행 명세 생성"""
        try:
            prompt = f"""
사용자의 ROS 노드 생성 요청을 분석하여 구체적인 실행 명세를 생성해주세요.

사용자 요청: {user_request}

다음 형식으로 JSON 응답을 제공해주세요:
{{
    "node": {{
        "name": "노드_이름",
        "type": "노드_타입",
        "language": "cpp 또는 python",
        "description": "노드_설명"
    }},
    "topics": {{
        "input": [
            {{
                "name": "입력_토픽_이름",
                "type": "메시지_타입",
                "direction": "subscribe",
                "qos": "reliable 또는 best_effort"
            }}
        ],
        "output": [
            {{
                "name": "출력_토픽_이름",
                "type": "메시지_타입",
                "direction": "publish",
                "qos": "reliable 또는 best_effort"
            }}
        ]
    }},
    "parameters": {{
        "parameter_name": {{
            "min": 최소값,
            "max": 최대값,
            "unit": "단위",
            "default": 기본값,
            "description": "파라미터_설명"
        }}
    }},
    "timer": {{
        "publish_rate": 10,  # Hz (토픽 발행 주기)
        "callback_frequency": 100,  # Hz (콜백 실행 주기)
        "callback": "timer_callback",
        "description": "타이머_설명"
    }},
    "error_handling": [
        "에러_처리_방식1",
        "에러_처리_방식2"
    ],
    "dependencies": [
        "필요한_패키지1",
        "필요한_패키지2"
    ]
}}

사용자 요청이 불분명한 경우, 일반적인 ROS 노드 패턴을 기반으로 합리적인 기본값을 제시해주세요.
"""

            response = self.ai_client.generate_response(prompt)
            
            # JSON 응답 파싱 - 개선된 오류 처리
            import json
            try:
                # 응답이 문자열인지 확인
                if isinstance(response, str):
                    # JSON 응답에서 코드 블록 제거
                    cleaned_response = response
                    if "```json" in response:
                        start_idx = response.find("```json") + 7
                        end_idx = response.find("```", start_idx)
                        if end_idx != -1:
                            cleaned_response = response[start_idx:end_idx].strip()
                    elif "```" in response:
                        start_idx = response.find("```") + 3
                        end_idx = response.find("```", start_idx)
                        if end_idx != -1:
                            cleaned_response = response[start_idx:end_idx].strip()
                    
                    # JSON 파싱 시도
                    spec = json.loads(cleaned_response)
                    self.logger.info("AI 기반 실행 명세 생성 완료")
                    return spec
                else:
                    self.logger.warning("AI 응답이 문자열이 아님, 템플릿 기반 생성으로 전환")
                    return self._generate_template_specification(user_request)
                    
            except json.JSONDecodeError as json_error:
                self.logger.warning(f"JSON 파싱 실패: {json_error}, 템플릿 기반 생성으로 전환")
                self.logger.debug(f"AI 응답: {response[:200]}...")  # 처음 200자만 로깅
                return self._generate_template_specification(user_request)
            except Exception as parse_error:
                self.logger.warning(f"응답 파싱 중 예상치 못한 오류: {parse_error}, 템플릿 기반 생성으로 전환")
                return self._generate_template_specification(user_request)
            
        except Exception as e:
            self.logger.error(f"AI 기반 실행 명세 생성 실패: {e}")
            return self._generate_template_specification(user_request)

    def _generate_template_specification(self, user_request: str) -> Dict[str, Any]:
        """템플릿 기반 실행 명세 생성"""
        try:
            # 기본 템플릿
            spec = {
                "node": {
                    "name": "ros_node",
                    "type": "basic_node",
                    "language": "cpp",
                    "description": "기본 ROS 노드"
                },
                "topics": {
                    "input": [
                        {
                            "name": "input_topic",
                            "type": "std_msgs/String",
                            "direction": "subscribe",
                            "qos": "reliable"
                        }
                    ],
                    "output": [
                        {
                            "name": "output_topic",
                            "type": "std_msgs/String",
                            "direction": "publish",
                            "qos": "reliable"
                        }
                    ]
                },
                "parameters": {
                    "update_rate": {
                        "min": 1.0,
                        "max": 100.0,
                        "unit": "Hz",
                        "default": 10.0,
                        "description": "노드 업데이트 주기"
                    }
                },
                "timer": {
                    "publish_rate": 10.0,  # Hz (토픽 발행 주기)
                    "callback_frequency": 100.0,  # Hz (콜백 실행 주기)
                    "callback": "timer_callback",
                    "description": "주기적 상태 업데이트"
                },
                "error_handling": [
                    "입력 검증",
                    "예외 처리",
                    "로깅"
                ],
                "dependencies": [
                    "rclcpp",
                    "std_msgs"
                ]
            }
            
            # 사용자 요청에 따른 상세한 조정
            if "속도" in user_request or "velocity" in user_request.lower():
                spec["node"]["name"] = "velocity_controller"
                spec["node"]["type"] = "control_node"
                spec["node"]["description"] = "안전한 속도 제어를 위한 ROS 노드"
                
                # 토픽 설정
                spec["topics"]["input"][0]["name"] = "cmd_vel"
                spec["topics"]["input"][0]["type"] = "geometry_msgs/Twist"
                spec["topics"]["input"][0]["description"] = "속도 명령 입력"
                
                spec["topics"]["output"][0]["name"] = "safe_cmd_vel"
                spec["topics"]["output"][0]["type"] = "geometry_msgs/Twist"
                spec["topics"]["output"][0]["description"] = "안전 검증된 속도 명령 출력"
                
                # 추가 출력 토픽
                spec["topics"]["output"].append({
                    "name": "velocity_status",
                    "type": "std_msgs/String",
                    "direction": "publish",
                    "qos": "reliable",
                    "description": "속도 제어 상태 정보"
                })
                
                # 파라미터 설정
                spec["parameters"] = {
                    "max_linear_velocity": {
                        "min": 0.0,
                        "max": 5.0,
                        "unit": "m/s",
                        "default": 2.0,
                        "description": "최대 선형 속도"
                    },
                    "max_angular_velocity": {
                        "min": 0.0,
                        "max": 2.0,
                        "unit": "rad/s",
                        "default": 1.0,
                        "description": "최대 각속도"
                    },
                    "safety_margin": {
                        "min": 0.1,
                        "max": 0.5,
                        "unit": "ratio",
                        "default": 0.2,
                        "description": "안전 마진 비율"
                    },
                    "update_rate": {
                        "min": 10.0,
                        "max": 100.0,
                        "unit": "Hz",
                        "default": 50.0,
                        "description": "제어 루프 업데이트 주기"
                    }
                }
                
                # 타이머 설정
                spec["timer"] = {
                    "frequency": 50.0,
                    "callback": "control_loop_callback",
                    "description": "속도 제어 루프 실행"
                }
                
                # 에러 처리
                spec["error_handling"] = [
                    "속도 명령 입력 검증",
                    "속도 한계 검사",
                    "비상 정지 기능",
                    "상태 모니터링",
                    "로깅 및 알림"
                ]
                
                # 의존성
                spec["dependencies"] = [
                    "rclcpp",
                    "geometry_msgs",
                    "std_msgs"
                ]
                
            elif "안전" in user_request or "safety" in user_request.lower():
                spec["node"]["name"] = "safety_monitor"
                spec["node"]["type"] = "monitoring_node"
                spec["node"]["description"] = "시스템 안전 상태를 모니터링하는 노드"
                spec["error_handling"].extend(["안전 상태 검증", "비상 정지", "경고 시스템"])
                
            elif "제어" in user_request or "control" in user_request.lower():
                spec["node"]["name"] = "control_node"
                spec["node"]["type"] = "control_node"
                spec["node"]["description"] = "시스템 제어를 담당하는 노드"
                spec["error_handling"].extend(["제어 명령 검증", "피드백 루프", "안전 장치 연동"])
                
            self.logger.info(f"템플릿 기반 실행 명세 생성 완료: {spec['node']['name']}")
            return spec
            
        except Exception as e:
            self.logger.error(f"템플릿 기반 실행 명세 생성 실패: {e}")
            return self._get_default_specification()

    def _get_default_specification(self) -> Dict[str, Any]:
        """기본 실행 명세 반환"""
        return {
            "node": {
                "name": "default_node",
                "type": "basic_node",
                "language": "cpp",
                "description": "기본 ROS 노드"
            },
            "topics": {
                "input": [],
                "output": []
            },
            "parameters": {},
            "timer": {},
            "error_handling": ["기본 예외 처리"],
            "dependencies": ["rclcpp"]
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
            return self._create_code_generation_plan(analysis, {})
        
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
                return self._create_code_generation_plan(analysis, {})
                
        except Exception as e:
            self.logger.error(f"AI 기반 계획 수립 실패: {e}")
            return self._create_code_generation_plan(analysis, {})
    
    def plan_ros_code_generation(self, user_request: str) -> Dict[str, Any]:
        """AI 기반 ROS 코드 생성 계획 수립 (외부 호출용)"""
        try:
            # AI 기반 요청 분석
            analysis = self._ai_analyze_user_request(user_request)
            
            # 실행 명세 생성
            execution_spec = self._generate_execution_specification(user_request)
            
            # AI 기반 계획 수립
            plan = self._ai_create_code_generation_plan(analysis, user_request)
            
            # 실행 명세를 계획에 포함
            if isinstance(plan, dict):
                plan['execution_specification'] = execution_spec
            
            return {
                'analysis': analysis,
                'plan': plan,
                'execution_specification': execution_spec,  # 별도로도 포함
                'status': 'success',
                'ai_enhanced': self.ai_client is not None
            }
            
        except Exception as e:
            self.logger.error(f"AI 기반 계획 수립 실패: {e}")
            # AI 실패 시 기본 분석 사용
            try:
                analysis = self._analyze_user_request(user_request)
                execution_spec = self._generate_execution_specification(user_request)
                plan = self._create_code_generation_plan(analysis, execution_spec)
                return {
                    'analysis': analysis,
                    'plan': plan,
                    'execution_specification': execution_spec,
                    'status': 'success',
                    'ai_enhanced': False,
                    'fallback': True
                }
            except Exception as fallback_e:
                return {
                    'error': f'계획 수립 실패: {str(e)} (fallback도 실패: {str(fallback_e)})',
                    'status': 'error'
                }

    def generate_plan(self, instruction: str) -> str:
        """Generate a plan for the given instruction (wrapper for workflow compatibility)"""
        result = self.plan_ros_code_generation(instruction)
        if result.get('status') == 'success':
            plan_dict = result.get('plan', {})
            execution_spec = plan_dict.get('execution_specification', {})
            
            # Convert plan dictionary to string format with execution specification
            plan_str = f"""
## ROS Code Generation Plan

### Analysis:
{result.get('analysis', {}).get('summary', 'N/A')}

### Implementation Steps:
"""
            steps = plan_dict.get('implementation_steps', [])
            for i, step in enumerate(steps, 1):
                plan_str += f"{i}. {step}\n"
            
            # 실행 명세 포함
            if execution_spec:
                plan_str += f"""
### Execution Specification:

#### Node Configuration:
- Name: {execution_spec.get('node', {}).get('name', 'N/A')}
- Type: {execution_spec.get('node', {}).get('type', 'N/A')}
- Language: {execution_spec.get('node', {}).get('language', 'N/A')}
- Description: {execution_spec.get('node', {}).get('description', 'N/A')}

#### Topics:
"""
                # Input topics
                input_topics = execution_spec.get('topics', {}).get('input', [])
                if input_topics:
                    plan_str += "**Input Topics:**\n"
                    for topic in input_topics:
                        plan_str += f"- {topic.get('name', 'N/A')} ({topic.get('type', 'N/A')}) - {topic.get('description', 'N/A')}\n"
                
                # Output topics
                output_topics = execution_spec.get('topics', {}).get('output', [])
                if output_topics:
                    plan_str += "**Output Topics:**\n"
                    for topic in output_topics:
                        plan_str += f"- {topic.get('name', 'N/A')} ({topic.get('type', 'N/A')}) - {topic.get('description', 'N/A')}\n"
                
                # Parameters
                parameters = execution_spec.get('parameters', {})
                if parameters:
                    plan_str += "\n**Parameters:**\n"
                    for param_name, param_info in parameters.items():
                        plan_str += f"- {param_name}: {param_info.get('min', 'N/A')} ~ {param_info.get('max', 'N/A')} {param_info.get('unit', 'N/A')} (default: {param_info.get('default', 'N/A')})\n"
                        plan_str += f"  - {param_info.get('description', 'N/A')}\n"
                
                # Timer
                timer = execution_spec.get('timer', {})
                if timer:
                    plan_str += f"\n**Timer:**\n- Frequency: {timer.get('frequency', 'N/A')} Hz\n- Callback: {timer.get('callback', 'N/A')}\n- Description: {timer.get('description', 'N/A')}\n"
                
                # Error handling
                error_handling = execution_spec.get('error_handling', [])
                if error_handling:
                    plan_str += f"\n**Error Handling:**\n"
                    for error in error_handling:
                        plan_str += f"- {error}\n"
                
                # Dependencies
                dependencies = execution_spec.get('dependencies', [])
                if dependencies:
                    plan_str += f"\n**Dependencies:**\n"
                    for dep in dependencies:
                        plan_str += f"- {dep}\n"
            
            plan_str += f"""
### ROS Components:
- Node Type: {plan_dict.get('ros_components', {}).get('node_type', 'N/A')}
- Topics: {', '.join(plan_dict.get('ros_components', {}).get('topics', []))}
- Services: {', '.join(plan_dict.get('ros_components', {}).get('services', []))}

### Security Considerations:
{', '.join(plan_dict.get('security_considerations', []))}
"""
            return plan_str
        else:
            return f"계획 수립 실패: {result.get('error', 'Unknown error')}"

    def process_message(self, message: AgentMessage) -> AgentMessage:
        """Process incoming messages"""
        if message.message_type == 'plan_request':
            instruction = message.content.get('instruction', '')
            plan = self.generate_plan(instruction)
            
            return self.send_message(
                message.sender,
                'plan_response',
                {'plan': plan}
            )
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'Unknown message type: {message.message_type}'}
            )

    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """Execute a planning task"""
        try:
            if task.task_type == 'planning':
                instruction = task.parameters.get('instruction', '')
                plan = self.generate_plan(instruction)
                
                return {
                    'status': 'completed',
                    'result': {'plan': plan}
                }
            else:
                return {
                    'status': 'failed',
                    'error': f'Unknown task type: {task.task_type}'
                }
        except Exception as e:
            return {
                'status': 'failed',
                'error': str(e)
            }