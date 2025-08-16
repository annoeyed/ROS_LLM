#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAG Guard Agent
Planner의 알고리즘을 RAG로 검증하고 보안 위험 요소를 식별하는 Agent
"""

import sys
import os
import re
from typing import Dict, Any, List, Optional
from .base_agent import BaseAgent, AgentMessage, AgentTask

# 상위 디렉토리 경로 추가 (rag_utils 모듈 접근용)
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class RAGGuardAgent(BaseAgent):
    """RAG 기반 보안 검증 및 가이드라인 제공 Agent"""
    
    def __init__(self, agent_id: str = "rag_guard_001"):
        super().__init__(agent_id, "RAG Guard Agent")
        
        # RAG 시스템 초기화
        self.cwe_rag = None
        self.cve_rag = None
        self.security_guidelines = None
        
        # AI 클라이언트 초기화
        self.ai_client = None
        
        # 보안 위험도 레벨
        self.risk_levels = {
            'critical': 4,
            'high': 3,
            'medium': 2,
            'low': 1
        }
        
        # 보안 검증 카테고리
        self.security_categories = [
            'authentication', 'authorization', 'input_validation',
            'memory_management', 'file_operations', 'race_conditions',
            'network_security', 'cryptography', 'logging', 'error_handling'
        ]
    
    def _initialize(self):
        """RAG Guard Agent 초기화"""
        super()._initialize()
        
        try:
            # RAG 시스템 로드
            self._load_rag_systems()
            self.logger.info("RAG 시스템 로드 완료")
            
            # AI 클라이언트 로드
            self._load_ai_client()
            self.logger.info("AI 클라이언트 로드 완료")
            
        except Exception as e:
            self.logger.error(f"시스템 로드 실패: {e}")
            self.status = 'error'
    
    def _load_rag_systems(self):
        """RAG 시스템 로드"""
        try:
            # CWE RAG 시스템 로드
            from rag_utils.cwe_rag import CWERAGSearch
            self.cwe_rag = CWERAGSearch()
            
            # RAG 시스템이 제대로 초기화되었는지 확인
            if not self.cwe_rag or not self.cwe_rag.index:
                self.logger.warning("CWE RAG 시스템 초기화 실패, 기본 검증만 사용")
                self.cwe_rag = None
            
            # CVE RAG 시스템 로드 (향후 구현)
            # from rag_utils.cve_rag import CVERAGSearch
            # self.cve_rag = CVERAGSearch()
            
            # 보안 가이드라인 로드
            from rag_utils.security_guidelines import SecurityGuidelineGenerator
            from rag_utils.cwe_database import CWEDatabase
            
            cwe_db = CWEDatabase()
            cwe_db.load_database()
            
            generator = SecurityGuidelineGenerator(cwe_db, self.cwe_rag)
            self.security_guidelines = generator.generate_ros_security_guidelines()
            
            self.logger.info(f"보안 가이드라인 로드 완료: {len(self.security_guidelines.get('categories', {}))}개 카테고리")
            
        except Exception as e:
            self.logger.error(f"RAG 시스템 로드 중 오류: {e}")
            self.cwe_rag = None
            # RAG 시스템 실패 시에도 계속 진행
    
    def _load_ai_client(self):
        """AI 클라이언트 로드"""
        try:
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
    
    def process_message(self, message: AgentMessage) -> AgentMessage:
        """메시지 처리 - 보안 검증 요청 처리"""
        if message.message_type == 'request':
            if 'verify_algorithm' in message.content:
                return self._handle_algorithm_verification(message)
            elif 'verify_code_security' in message.content:
                return self._handle_code_security_verification(message)
            elif 'get_security_guidelines' in message.content:
                return self._handle_guidelines_request(message)
            elif 'analyze_security_risks' in message.content:
                return self._handle_risk_analysis_request(message)
            else:
                return self._handle_general_request(message)
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'지원하지 않는 메시지 타입: {message.message_type}'}
            )
    
    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """작업 실행 - 보안 검증 및 가이드라인 제공"""
        self.status = 'busy'
        self.current_task = task
        
        try:
            if task.task_type == 'verify_algorithm':
                result = self._verify_algorithm_security(task.parameters)
            elif task.task_type == 'verify_code_security':
                result = self._verify_code_security(task.parameters)
            elif task.task_type == 'generate_security_checklist':
                result = self._generate_security_checklist(task.parameters)
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
    
    def _handle_algorithm_verification(self, message: AgentMessage) -> AgentMessage:
        """알고리즘 보안 검증 요청 처리"""
        algorithm = message.content.get('algorithm', '')
        component = message.content.get('component', 'general')
        
        verification_result = self._verify_algorithm_security({
            'algorithm': algorithm,
            'component': component
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'verification_result': verification_result,
                'component': component,
                'status': 'success'
            }
        )
    
    def _handle_code_security_verification(self, message: AgentMessage) -> AgentMessage:
        """코드 보안 검증 요청 처리"""
        code_snippet = message.content.get('code_snippet', '')
        component = message.content.get('component', 'general')
        
        verification_result = self._verify_code_security({
            'code_snippet': code_snippet,
            'component': component
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'verification_result': verification_result,
                'component': component,
                'status': 'success'
            }
        )
    
    def _handle_guidelines_request(self, message: AgentMessage) -> AgentMessage:
        """보안 가이드라인 요청 처리"""
        category = message.content.get('category', 'all')
        component = message.content.get('component', 'all')
        
        guidelines = self._get_relevant_guidelines(category, component)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'guidelines': guidelines,
                'category': category,
                'component': component,
                'status': 'success'
            }
        )
    
    def _handle_risk_analysis_request(self, message: AgentMessage) -> AgentMessage:
        """보안 위험도 분석 요청 처리"""
        content = message.content.get('content', '')
        content_type = message.content.get('content_type', 'algorithm')  # algorithm or code
        component = message.content.get('component', 'general')
        
        if content_type == 'algorithm':
            risk_analysis = self._analyze_algorithm_security_risks(content, component)
        else:
            risk_analysis = self._analyze_code_security_risks(content, component)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'risk_analysis': risk_analysis,
                'content_type': content_type,
                'component': component,
                'status': 'success'
            }
        )
    
    def _handle_general_request(self, message: AgentMessage) -> AgentMessage:
        """일반 요청 처리"""
        return self.send_message(
            message.sender,
            'response',
            {
                'message': 'RAG Guard Agent가 요청을 처리했습니다.',
                'content': message.content,
                'status': 'success'
            }
        )
    
    def _verify_algorithm_security(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """알고리즘 보안 검증"""
        algorithm = parameters.get('algorithm', '')
        component = parameters.get('component', 'general')
        
        if not algorithm:
            return {'error': '알고리즘이 제공되지 않았습니다.'}
        
        # RAG를 사용한 보안 검증
        verification_result = self._rag_based_security_verification(algorithm, component)
        
        return {
            'verification_passed': verification_result['overall_status'] == 'pass',
            'overall_status': verification_result['overall_status'],
            'security_score': verification_result['security_score'],
            'issues_found': verification_result['issues_found'],
            'recommendations': verification_result['recommendations'],
            'component': component
        }
    
    def _verify_code_security(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """코드 보안 검증"""
        code_snippet = parameters.get('code_snippet', '')
        component = parameters.get('component', 'general')
        
        if not code_snippet:
            return {'error': '코드 스니펫이 제공되지 않았습니다.'}
        
        # RAG를 사용한 코드 보안 검증
        verification_result = self._rag_based_code_verification(code_snippet, component)
        
        return {
            'verification_passed': verification_result['overall_status'] == 'pass',
            'overall_status': verification_result['overall_status'],
            'security_score': verification_result['security_score'],
            'vulnerabilities_found': verification_result['vulnerabilities_found'],
            'recommendations': verification_result['recommendations'],
            'component': component
        }
    
    def _rag_based_security_verification(self, algorithm: str, component: str) -> Dict[str, Any]:
        """AI + RAG 기반 알고리즘 보안 검증"""
        issues_found = []
        security_score = 100  # 초기 점수
        
        # 1. 기본 패턴 기반 검증
        basic_verification = self._basic_pattern_verification(algorithm, component)
        issues_found.extend(basic_verification['issues'])
        security_score -= basic_verification['score_deduction']
        
        # 2. RAG 검색을 통한 보안 검증
        rag_verification = self._perform_rag_security_search(algorithm, component)
        issues_found.extend(rag_verification.get('issues', []))
        security_score -= rag_verification.get('score_deduction', 0)
        
        # 3. AI 기반 고급 보안 분석
        if self.ai_client:
            ai_verification = self._ai_based_security_analysis(algorithm, component, issues_found)
            issues_found.extend(ai_verification.get('issues', []))
            security_score -= ai_verification.get('score_deduction', 0)
            
            # AI 권장사항 추가
            ai_recommendations = ai_verification.get('ai_recommendations', [])
        else:
            ai_recommendations = []
        
        # 전체 상태 결정
        if security_score >= 80:
            overall_status = 'pass'
        elif security_score >= 60:
            overall_status = 'warning'
        else:
            overall_status = 'fail'
        
        # AI 기반 권장사항과 기본 권장사항 결합
        basic_recommendations = self._generate_security_recommendations(issues_found, component)
        all_recommendations = basic_recommendations + ai_recommendations
        
        return {
            'overall_status': overall_status,
            'security_score': max(0, security_score),
            'issues_found': issues_found,
            'recommendations': all_recommendations,
            'ai_enhanced': self.ai_client is not None,
            'verification_methods': ['pattern', 'rag', 'ai'] if self.ai_client else ['pattern', 'rag']
        }
    
    def _basic_pattern_verification(self, algorithm: str, component: str) -> Dict[str, Any]:
        """기본 패턴 기반 보안 검증"""
        issues_found = []
        score_deduction = 0
        
        # 알고리즘에서 보안 위험 패턴 검색
        security_patterns = {
            'authentication_bypass': [r'bypass.*auth', r'skip.*verification', r'admin.*override'],
            'insecure_input': [r'user.*input.*direct', r'no.*validation', r'unsafe.*input'],
            'privilege_escalation': [r'elevate.*privilege', r'root.*access', r'escalate.*permission'],
            'data_exposure': [r'expose.*data', r'log.*password', r'debug.*info'],
            'weak_encryption': [r'plain.*text', r'no.*encryption', r'weak.*hash']
        }
        
        for risk_type, patterns in security_patterns.items():
            for pattern in patterns:
                if re.search(pattern, algorithm, re.IGNORECASE):
                    issue_info = self._get_issue_info(risk_type, component)
                    issues_found.append(issue_info)
                    score_deduction += issue_info.get('severity_score', 10)
        
        return {
            'issues': issues_found,
            'score_deduction': score_deduction
        }
    
    def _ai_based_security_analysis(self, algorithm: str, component: str, existing_issues: List[Dict[str, Any]]) -> Dict[str, Any]:
        """AI 기반 고급 보안 분석"""
        try:
            # AI 프롬프트 구성
            ai_prompt = f"""
            다음 ROS 알고리즘의 보안을 심층 분석하세요:
            
            알고리즘:
            {algorithm}
            
            컴포넌트: {component}
            
            이미 발견된 이슈:
            {existing_issues}
            
            다음을 고려하여 분석하세요:
            1. ROS 특화 보안 위험 요소
            2. 실시간 시스템 보안 고려사항
            3. 로봇 시스템의 물리적 보안 위험
            4. 네트워크 통신 보안
            5. 데이터 무결성 및 프라이버시
            
            JSON 형식으로 응답하세요:
            {{
                "additional_issues": [
                    {{
                        "type": "AI_감지_이슈",
                        "title": "제목",
                        "description": "설명",
                        "severity_score": 0-25,
                        "mitigation": "완화 방안",
                        "ai_confidence": 0.0-1.0
                    }}
                ],
                "ai_recommendations": ["AI 권장사항1", "AI 권장사항2"],
                "score_deduction": 0
            }}
            """
            
            # AI 분석 수행
            ai_response = self.ai_client.analyze_content(algorithm, "security")
            
            if isinstance(ai_response, dict) and 'additional_issues' in ai_response:
                # AI 응답 파싱 성공
                additional_issues = ai_response.get('additional_issues', [])
                ai_recommendations = ai_response.get('ai_recommendations', [])
                score_deduction = ai_response.get('score_deduction', 0)
                
                # AI 신뢰도가 높은 이슈만 필터링
                high_confidence_issues = [
                    issue for issue in additional_issues 
                    if issue.get('ai_confidence', 0) > 0.7
                ]
                
                return {
                    'issues': high_confidence_issues,
                    'ai_recommendations': ai_recommendations,
                    'score_deduction': score_deduction
                }
            else:
                # AI 응답 파싱 실패 시 기본 분석
                return {
                    'issues': [],
                    'ai_recommendations': ["AI 분석을 통한 추가 보안 검토 권장"],
                    'score_deduction': 0
                }
                
        except Exception as e:
            self.logger.error(f"AI 기반 보안 분석 실패: {e}")
            return {
                'issues': [],
                'ai_recommendations': ["AI 분석 중 오류 발생"],
                'score_deduction': 0
            }
    
    def _rag_based_code_verification(self, code_snippet: str, component: str) -> Dict[str, Any]:
        """AI + RAG 기반 코드 보안 검증"""
        vulnerabilities_found = []
        security_score = 100
        
        # 1. 기본 패턴 기반 검증
        basic_verification = self._basic_code_pattern_verification(code_snippet, component)
        vulnerabilities_found.extend(basic_verification['vulnerabilities'])
        security_score -= basic_verification['score_deduction']
        
        # 2. RAG 검색을 통한 보안 검증
        rag_verification = self._perform_rag_code_verification(code_snippet, component)
        vulnerabilities_found.extend(rag_verification.get('vulnerabilities', []))
        security_score -= rag_verification.get('score_deduction', 0)
        
        # 3. AI 기반 고급 코드 보안 분석
        if self.ai_client:
            ai_verification = self._ai_based_code_analysis(code_snippet, component, vulnerabilities_found)
            vulnerabilities_found.extend(ai_verification.get('vulnerabilities', []))
            security_score -= ai_verification.get('score_deduction', 0)
            
            # AI 권장사항 추가
            ai_recommendations = ai_verification.get('ai_recommendations', [])
        else:
            ai_recommendations = []
        
        # 전체 상태 결정
        if security_score >= 80:
            overall_status = 'pass'
        elif security_score >= 60:
            overall_status = 'warning'
        else:
            overall_status = 'fail'
        
        # AI 기반 권장사항과 기본 권장사항 결합
        basic_recommendations = self._generate_code_security_recommendations(vulnerabilities_found, component)
        all_recommendations = basic_recommendations + ai_recommendations
        
        return {
            'overall_status': overall_status,
            'security_score': max(0, security_score),
            'vulnerabilities_found': vulnerabilities_found,
            'recommendations': all_recommendations,
            'ai_enhanced': self.ai_client is not None,
            'verification_methods': ['pattern', 'rag', 'ai'] if self.ai_client else ['pattern', 'rag']
        }
    
    def _basic_code_pattern_verification(self, code_snippet: str, component: str) -> Dict[str, Any]:
        """기본 코드 패턴 기반 보안 검증"""
        vulnerabilities_found = []
        score_deduction = 0
        
        # 코드에서 보안 취약점 패턴 검색
        vulnerability_patterns = {
            'sql_injection': [r'execute\s*\(\s*[^)]*\+', r'query\s*=\s*[^=]*\+'],
            'command_injection': [r'os\.system\s*\(', r'subprocess\.call\s*\(', r'exec\s*\('],
            'path_traversal': [r'\.\./', r'\.\.\\', r'file://'],
            'hardcoded_credentials': [r'password\s*=\s*["\'][^"\']+["\']', r'api_key\s*=\s*["\'][^"\']+["\']'],
            'weak_encryption': [r'MD5\s*\(', r'SHA1\s*\(', r'base64\s*\.\s*encode'],
            'insecure_communication': [r'http://', r'ftp://', r'telnet://']
        }
        
        for vuln_type, patterns in vulnerability_patterns.items():
            for pattern in patterns:
                if re.search(pattern, code_snippet, re.IGNORECASE):
                    vuln_info = self._get_vulnerability_info(vuln_type, component)
                    vulnerabilities_found.append(vuln_info)
                    score_deduction += vuln_info.get('severity_score', 15)
        
        return {
            'vulnerabilities': vulnerabilities_found,
            'score_deduction': score_deduction
        }
    
    def _ai_based_code_analysis(self, code_snippet: str, component: str, existing_vulnerabilities: List[Dict[str, Any]]) -> Dict[str, Any]:
        """AI 기반 고급 코드 보안 분석"""
        try:
            # AI 프롬프트 구성
            ai_prompt = f"""
            다음 ROS Python 코드의 보안을 심층 분석하세요:
            
            코드:
            {code_snippet}
            
            컴포넌트: {component}
            
            이미 발견된 취약점:
            {existing_vulnerabilities}
            
            다음을 고려하여 분석하세요:
            1. ROS 2 특화 보안 위험 요소
            2. Python 보안 모범 사례
            3. 실시간 시스템 보안 고려사항
            4. 메모리 관리 및 리소스 보안
            5. 비동기 처리 보안
            6. ROS 토픽/서비스 보안
            
            JSON 형식으로 응답하세요:
            {{
                "additional_vulnerabilities": [
                    {{
                        "type": "AI_감지_취약점",
                        "title": "제목",
                        "description": "설명",
                        "severity_score": 0-25,
                        "mitigation": "완화 방안",
                        "ai_confidence": 0.0-1.0,
                        "cwe_id": "CWE-XXX"
                    }}
                ],
                "ai_recommendations": ["AI 권장사항1", "AI 권장사항2"],
                "score_deduction": 0
            }}
            """
            
            # AI 분석 수행
            ai_response = self.ai_client.analyze_content(code_snippet, "security")
            
            if isinstance(ai_response, dict) and 'additional_vulnerabilities' in ai_response:
                # AI 응답 파싱 성공
                additional_vulns = ai_response.get('additional_vulnerabilities', [])
                ai_recommendations = ai_response.get('ai_recommendations', [])
                score_deduction = ai_response.get('score_deduction', 0)
                
                # AI 신뢰도가 높은 취약점만 필터링
                high_confidence_vulns = [
                    vuln for vuln in additional_vulns 
                    if vuln.get('ai_confidence', 0) > 0.7
                ]
                
                return {
                    'vulnerabilities': high_confidence_vulns,
                    'ai_recommendations': ai_recommendations,
                    'score_deduction': score_deduction
                }
            else:
                # AI 응답 파싱 실패 시 기본 분석
                return {
                    'vulnerabilities': [],
                    'ai_recommendations': ["AI 분석을 통한 추가 보안 검토 권장"],
                    'score_deduction': 0
                }
                
        except Exception as e:
            self.logger.error(f"AI 기반 코드 분석 실패: {e}")
            return {
                'vulnerabilities': [],
                'ai_recommendations': ["AI 분석 중 오류 발생"],
                'score_deduction': 0
            }
    
    def _perform_rag_security_search(self, algorithm: str, component: str) -> Dict[str, Any]:
        """RAG를 사용한 알고리즘 보안 검색"""
        try:
            # RAG 시스템이 초기화되었는지 확인
            if not self.cwe_rag:
                self.logger.warning("CWE RAG 시스템이 초기화되지 않았습니다.")
                return {'issues': [], 'score_deduction': 0}
            
            # CWE RAG 검색
            search_results = self.cwe_rag.search(algorithm, top_k=5)
            
            issues = []
            score_deduction = 0
            
            for result in search_results:
                if result.get('score', 0) > 0.7:  # 높은 유사도 결과만
                    cwe_info = result.get('metadata', {})
                    if cwe_info:
                        issue = {
                            'type': 'rag_detected',
                            'title': f"RAG 검색 결과: {cwe_info.get('cwe_id', 'Unknown')}",
                            'description': cwe_info.get('description', ''),
                            'severity_score': 5,
                            'source': 'CWE RAG',
                            'cwe_id': cwe_info.get('cwe_id', '')
                        }
                        issues.append(issue)
                        score_deduction += 5
            
            return {
                'issues': issues,
                'score_deduction': score_deduction
            }
            
        except Exception as e:
            self.logger.error(f"RAG 보안 검색 실패: {e}")
            return {'issues': [], 'score_deduction': 0}
    
    def _perform_rag_code_verification(self, code_snippet: str, component: str) -> Dict[str, Any]:
        """RAG를 사용한 코드 보안 검증"""
        try:
            # CWE RAG 검색
            search_results = self.cwe_rag.search(code_snippet, top_k=5)
            
            vulnerabilities = []
            score_deduction = 0
            
            for result in search_results:
                if result.get('score', 0) > 0.7:  # 높은 유사도 결과만
                    cwe_info = result.get('metadata', {})
                    if cwe_info:
                        vuln = {
                            'type': 'rag_detected',
                            'title': f"RAG 검색 결과: {cwe_info.get('cwe_id', 'Unknown')}",
                            'description': cwe_info.get('description', ''),
                            'severity_score': 10,
                            'source': 'CWE RAG',
                            'cwe_id': cwe_info.get('cwe_id', ''),
                            'mitigation': cwe_info.get('mitigation', '')
                        }
                        vulnerabilities.append(vuln)
                        score_deduction += 10
            
            return {
                'vulnerabilities': vulnerabilities,
                'score_deduction': score_deduction
            }
            
        except Exception as e:
            self.logger.error(f"RAG 코드 검증 실패: {e}")
            return {'vulnerabilities': [], 'score_deduction': 0}
    
    def _get_issue_info(self, issue_type: str, component: str) -> Dict[str, Any]:
        """이슈 유형별 상세 정보"""
        issue_descriptions = {
            'authentication_bypass': {
                'title': '인증 우회 위험',
                'description': '알고리즘에서 인증 검증을 우회할 수 있는 경로가 존재함',
                'severity_score': 20,
                'mitigation': '모든 보안 검증 단계를 필수로 설정'
            },
            'insecure_input': {
                'title': '안전하지 않은 입력 처리',
                'description': '사용자 입력에 대한 적절한 검증이 부족함',
                'severity_score': 15,
                'mitigation': '입력 검증 및 정제 로직 추가'
            },
            'privilege_escalation': {
                'title': '권한 상승 위험',
                'description': '알고리즘에서 권한을 상승시킬 수 있는 경로가 존재함',
                'severity_score': 25,
                'mitigation': '최소 권한 원칙 적용 및 권한 검증 강화'
            },
            'data_exposure': {
                'title': '데이터 노출 위험',
                'description': '민감한 정보가 로그나 디버그 정보에 노출될 수 있음',
                'severity_score': 15,
                'mitigation': '민감 정보 마스킹 및 로깅 정책 수립'
            },
            'weak_encryption': {
                'title': '약한 암호화',
                'description': '알고리즘에서 취약한 암호화 방식 사용',
                'severity_score': 20,
                'mitigation': '강력한 암호화 알고리즘 사용'
            }
        }
        
        return issue_descriptions.get(issue_type, {
            'title': '알 수 없는 보안 이슈',
            'description': '알고리즘에서 보안 위험이 감지됨',
            'severity_score': 10,
            'mitigation': '보안 전문가 검토 필요'
        })
    
    def _get_vulnerability_info(self, vuln_type: str, component: str) -> Dict[str, Any]:
        """취약점 유형별 상세 정보"""
        vuln_descriptions = {
            'sql_injection': {
                'title': 'SQL 인젝션',
                'description': '사용자 입력이 SQL 쿼리에 직접 포함되어 있음',
                'severity_score': 20,
                'mitigation': '매개변수화된 쿼리 사용'
            },
            'command_injection': {
                'title': '명령어 인젝션',
                'description': '사용자 입력이 시스템 명령어에 직접 사용됨',
                'severity_score': 25,
                'mitigation': '명령어 실행 방지 또는 입력 검증'
            },
            'path_traversal': {
                'title': '경로 순회 공격',
                'description': '사용자 입력이 파일 경로에 직접 사용됨',
                'severity_score': 15,
                'mitigation': '경로 검증 및 정규화'
            },
            'hardcoded_credentials': {
                'title': '하드코딩된 자격 증명',
                'description': '코드에 비밀번호나 API 키가 직접 포함되어 있음',
                'severity_score': 15,
                'mitigation': '환경 변수나 보안 저장소 사용'
            },
            'weak_encryption': {
                'title': '약한 암호화',
                'description': '취약한 암호화 알고리즘 사용',
                'severity_score': 20,
                'mitigation': '강력한 암호화 알고리즘 사용'
            },
            'insecure_communication': {
                'title': '안전하지 않은 통신',
                'description': '암호화되지 않은 통신 프로토콜 사용',
                'severity_score': 15,
                'mitigation': 'HTTPS, WSS 등 보안 프로토콜 사용'
            }
        }
        
        return vuln_descriptions.get(vuln_type, {
            'title': '알 수 없는 보안 취약점',
            'description': '코드에서 보안 취약점이 감지됨',
            'severity_score': 10,
            'mitigation': '보안 전문가 검토 필요'
        })
    
    def _generate_security_recommendations(self, issues: List[Dict[str, Any]], component: str) -> List[str]:
        """보안 권장사항 생성"""
        recommendations = []
        
        for issue in issues:
            if issue.get('mitigation'):
                recommendations.append(f"{issue['title']}: {issue['mitigation']}")
        
        # 컴포넌트별 일반적인 보안 권장사항 추가
        if component == 'rclpy/rclcpp':
            recommendations.extend([
                'ROS 2 보안 기능 활성화',
                'DDS 보안 설정 구성',
                '인증 및 권한 관리 구현'
            ])
        elif component == 'tf2':
            recommendations.extend([
                '변환 데이터 검증',
                '좌표계 보안 검증'
            ])
        elif component == 'navigation':
            recommendations.extend([
                '경로 계획 보안 검증',
                '충돌 회피 알고리즘 검증'
            ])
        
        return recommendations
    
    def _generate_code_security_recommendations(self, vulnerabilities: List[Dict[str, Any]], component: str) -> List[str]:
        """코드 보안 권장사항 생성"""
        recommendations = []
        
        for vuln in vulnerabilities:
            if vuln.get('mitigation'):
                recommendations.append(f"{vuln['title']}: {vuln['mitigation']}")
        
        # 일반적인 코드 보안 권장사항
        recommendations.extend([
            '정적 분석 도구 활용',
            '보안 코딩 표준 준수',
            '정기적인 보안 테스트 수행',
            '코드 리뷰 프로세스 강화'
        ])
        
        return recommendations
    
    def _get_relevant_guidelines(self, category: str = 'all', component: str = 'all') -> Dict[str, Any]:
        """관련 보안 가이드라인 조회"""
        if not self.security_guidelines:
            return {'error': '보안 가이드라인이 로드되지 않았습니다.'}
        
        result = {}
        
        # 카테고리별 가이드라인
        if category == 'all':
            result['categories'] = self.security_guidelines.get('categories', {})
        else:
            result['categories'] = {
                category: self.security_guidelines.get('categories', {}).get(category, {})
            }
        
        # 컴포넌트별 가이드라인
        if component == 'all':
            result['components'] = self.security_guidelines.get('components', {})
        else:
            result['components'] = {
                component: self.security_guidelines.get('components', {}).get(component, {})
            }
        
        return result
    
    def _analyze_algorithm_security_risks(self, algorithm: str, component: str) -> Dict[str, Any]:
        """알고리즘 보안 위험도 분석"""
        return self._rag_based_security_verification(algorithm, component)
    
    def _analyze_code_security_risks(self, code_snippet: str, component: str) -> Dict[str, Any]:
        """코드 보안 위험도 분석"""
        return self._rag_based_code_verification(code_snippet, component)
    
    def _generate_security_checklist(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """보안 체크리스트 생성"""
        component = parameters.get('component', 'general')
        phase = parameters.get('phase', 'development')
        
        if not self.security_guidelines:
            return {'error': '보안 가이드라인이 로드되지 않았습니다.'}
        
        checklist = self.security_guidelines.get('checklist', {})
        
        if component == 'all':
            return checklist
        
        # 특정 컴포넌트에 대한 체크리스트 생성
        component_guidelines = self.security_guidelines.get('components', {}).get(component, {})
        
        if not component_guidelines:
            return {'error': f'컴포넌트 {component}에 대한 가이드라인이 없습니다.'}
        
        # 컴포넌트별 맞춤 체크리스트 생성
        custom_checklist = {
            'component': component,
            'phase': phase,
            'risk_level': component_guidelines.get('risk_level', 'Unknown'),
            'critical_vulnerabilities': component_guidelines.get('critical_vulnerabilities', []),
            'general_security_items': checklist
        }
        
        return custom_checklist
    
    def get_status(self) -> Dict[str, Any]:
        """Agent 상태 조회"""
        return {
            'agent_id': self.agent_id,
            'agent_name': self.agent_name,
            'status': self.status,
            'message_queue_length': len(self.message_queue),
            'task_history_length': len(self.task_history),
            'current_task': self.current_task.task_id if self.current_task else None,
            'rag_systems_loaded': {
                'cwe_rag': self.cwe_rag is not None,
                'cve_rag': self.cve_rag is not None,
                'security_guidelines': self.security_guidelines is not None
            },
            'ai_client': {
                'loaded': self.ai_client is not None,
                'type': self.ai_client.__class__.__name__ if self.ai_client else None,
                'enhanced_analysis': self.ai_client is not None
            }
        }
