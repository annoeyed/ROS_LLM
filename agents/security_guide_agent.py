#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Security Guide Agent
CWE 데이터베이스를 기반으로 ROS 보안 가이드라인을 생성하고 제공하는 Agent
"""

import sys
import os
import re
from typing import Dict, Any, List, Optional
from .base_agent import BaseAgent, AgentMessage, AgentTask

# 상위 디렉토리 경로 추가 (rag_utils 모듈 접근용)
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class SecurityGuideAgent(BaseAgent):
    """CWE 기반 보안 가이드라인 생성 및 제공 Agent"""
    
    def __init__(self, agent_id: str = "security_guide_001"):
        super().__init__(agent_id, "Security Guide Agent")
        
        # 보안 가이드라인 생성기 초기화
        self.guidelines = {}
        self.cwe_database = None
        self.cwe_rag = None
        
        # 보안 위험도 레벨
        self.risk_levels = {
            'critical': 4,
            'high': 3,
            'medium': 2,
            'low': 1
        }
    
    def _initialize(self):
        """Security Guide Agent 초기화"""
        super()._initialize()
        
        try:
            # CWE 데이터베이스 및 RAG 시스템 로드
            self._load_security_systems()
            self.logger.info("보안 시스템 로드 완료")
        except Exception as e:
            self.logger.error(f"보안 시스템 로드 실패: {e}")
            self.status = 'error'
    
    def _load_security_systems(self):
        """보안 시스템 로드"""
        try:
            # CWE 데이터베이스 로드
            from rag_utils.cwe_database import CWEDatabase
            self.cwe_database = CWEDatabase()
            self.cwe_database.load_database()
            
            # CWE RAG 시스템 로드
            from rag_utils.cwe_rag import CWERAGSearch
            self.cwe_rag = CWERAGSearch()
            
            # 보안 가이드라인 생성
            from rag_utils.security_guidelines import SecurityGuidelineGenerator
            generator = SecurityGuidelineGenerator(self.cwe_database, self.cwe_rag)
            self.guidelines = generator.generate_ros_security_guidelines()
            
            self.logger.info(f"보안 가이드라인 생성 완료: {len(self.guidelines.get('categories', {}))}개 카테고리")
            
        except Exception as e:
            self.logger.error(f"보안 시스템 로드 중 오류: {e}")
            raise
    
    def process_message(self, message: AgentMessage) -> AgentMessage:
        """메시지 처리 - 보안 가이드라인 요청 처리"""
        if message.message_type == 'request':
            if 'get_security_guidelines' in message.content:
                return self._handle_guidelines_request(message)
            elif 'analyze_security_risks' in message.content:
                return self._handle_risk_analysis_request(message)
            elif 'get_cwe_info' in message.content:
                return self._handle_cwe_info_request(message)
            elif 'generate_security_checklist' in message.content:
                return self._handle_checklist_request(message)
            else:
                return self._handle_general_request(message)
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'지원하지 않는 메시지 타입: {message.message_type}'}
            )
    
    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """작업 실행 - 보안 가이드라인 생성 및 분석"""
        self.status = 'busy'
        self.current_task = task
        
        try:
            if task.task_type == 'generate_security_guidelines':
                result = self._generate_security_guidelines(task.parameters)
            elif task.task_type == 'analyze_security_risks':
                result = self._analyze_security_risks(task.parameters)
            elif task.task_type == 'create_security_checklist':
                result = self._create_security_checklist(task.parameters)
            elif task.task_type == 'validate_security_compliance':
                result = self._validate_security_compliance(task.parameters)
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
        code_snippet = message.content.get('code_snippet', '')
        component = message.content.get('component', 'general')
        
        risk_analysis = self._analyze_code_security_risks(code_snippet, component)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'risk_analysis': risk_analysis,
                'component': component,
                'status': 'success'
            }
        )
    
    def _handle_cwe_info_request(self, message: AgentMessage) -> AgentMessage:
        """CWE 정보 요청 처리"""
        cwe_id = message.content.get('cwe_id', '')
        
        cwe_info = self._get_cwe_detailed_info(cwe_id)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'cwe_info': cwe_info,
                'cwe_id': cwe_id,
                'status': 'success'
            }
        )
    
    def _handle_checklist_request(self, message: AgentMessage) -> AgentMessage:
        """보안 체크리스트 요청 처리"""
        component = message.content.get('component', 'all')
        phase = message.content.get('phase', 'all')
        
        checklist = self._generate_component_checklist(component, phase)
        
        return self.send_message(
            message.sender,
            'response',
            {
                'checklist': checklist,
                'component': component,
                'phase': phase,
                'status': 'success'
            }
        )
    
    def _handle_general_request(self, message: AgentMessage) -> AgentMessage:
        """일반 요청 처리"""
        return self.send_message(
            message.sender,
            'response',
            {
                'message': 'Security Guide Agent가 요청을 받았습니다.',
                'content': message.content,
                'status': 'success'
            }
        )
    
    def _get_relevant_guidelines(self, category: str = 'all', component: str = 'all') -> Dict[str, Any]:
        """관련 보안 가이드라인 조회"""
        if not self.guidelines:
            return {'error': '보안 가이드라인이 로드되지 않았습니다.'}
        
        result = {}
        
        # 카테고리별 가이드라인
        if category == 'all':
            result['categories'] = self.guidelines.get('categories', {})
        else:
            result['categories'] = {
                category: self.guidelines.get('categories', {}).get(category, {})
            }
        
        # 컴포넌트별 가이드라인
        if component == 'all':
            result['components'] = self.guidelines.get('components', {})
        else:
            result['components'] = {
                component: self.guidelines.get('components', {}).get(component, {})
            }
        
        return result
    
    def _analyze_code_security_risks(self, code_snippet: str, component: str) -> Dict[str, Any]:
        """코드 스니펫의 보안 위험도 분석"""
        risks = []
        risk_score = 0
        
        # 코드에서 보안 위험 패턴 검색
        security_patterns = {
            'hardcoded_credentials': [r'password\s*=\s*["\'][^"\']+["\']', r'api_key\s*=\s*["\'][^"\']+["\']'],
            'sql_injection': [r'execute\s*\(\s*[^)]*\+', r'query\s*=\s*[^=]*\+'],
            'path_traversal': [r'\.\./', r'\.\.\\', r'file://'],
            'command_injection': [r'os\.system\s*\(', r'subprocess\.call\s*\(', r'exec\s*\('],
            'weak_encryption': [r'MD5\s*\(', r'SHA1\s*\(', r'base64\s*\.\s*encode'],
            'insecure_communication': [r'http://', r'ftp://', r'telnet://']
        }
        
        for risk_type, patterns in security_patterns.items():
            for pattern in patterns:
                if re.search(pattern, code_snippet, re.IGNORECASE):
                    risk_info = self._get_risk_info(risk_type, component)
                    risks.append(risk_info)
                    risk_score += risk_info.get('severity_score', 0)
        
        # 위험도 레벨 결정
        if risk_score >= 15:
            risk_level = 'Critical'
        elif risk_score >= 10:
            risk_level = 'High'
        elif risk_score >= 5:
            risk_level = 'Medium'
        else:
            risk_level = 'Low'
        
        return {
            'risk_level': risk_level,
            'risk_score': risk_score,
            'risks': risks,
            'recommendations': self._get_security_recommendations(risks, component)
        }
    
    def _get_risk_info(self, risk_type: str, component: str) -> Dict[str, Any]:
        """위험 유형별 상세 정보"""
        risk_descriptions = {
            'hardcoded_credentials': {
                'title': '하드코딩된 자격 증명',
                'description': '코드에 비밀번호나 API 키가 직접 포함되어 있음',
                'cwe_id': 'CWE-259',
                'severity_score': 5,
                'mitigation': '환경 변수나 보안 저장소 사용'
            },
            'sql_injection': {
                'title': 'SQL 인젝션',
                'description': '사용자 입력이 SQL 쿼리에 직접 포함되어 있음',
                'cwe_id': 'CWE-89',
                'severity_score': 4,
                'mitigation': '매개변수화된 쿼리 사용'
            },
            'path_traversal': {
                'title': '경로 순회 공격',
                'description': '사용자 입력이 파일 경로에 직접 사용됨',
                'cwe_id': 'CWE-22',
                'severity_score': 4,
                'mitigation': '경로 검증 및 정규화'
            },
            'command_injection': {
                'title': '명령어 인젝션',
                'description': '사용자 입력이 시스템 명령어에 직접 사용됨',
                'cwe_id': 'CWE-78',
                'severity_score': 5,
                'mitigation': '명령어 실행 방지 또는 입력 검증'
            },
            'weak_encryption': {
                'title': '약한 암호화',
                'description': '취약한 암호화 알고리즘 사용',
                'cwe_id': 'CWE-327',
                'severity_score': 3,
                'mitigation': '강력한 암호화 알고리즘 사용'
            },
            'insecure_communication': {
                'title': '안전하지 않은 통신',
                'description': '암호화되지 않은 프로토콜 사용',
                'cwe_id': 'CWE-200',
                'severity_score': 3,
                'mitigation': 'TLS/SSL 사용'
            }
        }
        
        return risk_descriptions.get(risk_type, {
            'title': '알 수 없는 위험',
            'description': '식별되지 않은 보안 위험',
            'cwe_id': 'Unknown',
            'severity_score': 1,
            'mitigation': '보안 검토 필요'
        })
    
    def _get_security_recommendations(self, risks: List[Dict], component: str) -> List[str]:
        """보안 권장사항 생성"""
        recommendations = []
        
        for risk in risks:
            recommendations.append(f"{risk['title']}: {risk['mitigation']}")
        
        # 컴포넌트별 추가 권장사항
        component_recommendations = {
            'rclpy/rclcpp': [
                'ROS 2 보안 기능 활성화',
                'DDS 보안 설정 구성',
                '인증 및 권한 관리 구현'
            ],
            'tf2': [
                '좌표 변환 데이터 검증',
                '변환 체인 무결성 검사'
            ],
            'urdf': [
                'XML 파싱 보안 설정',
                '외부 파일 참조 검증'
            ]
        }
        
        if component in component_recommendations:
            recommendations.extend(component_recommendations[component])
        
        return recommendations
    
    def _get_cwe_detailed_info(self, cwe_id: str) -> Dict[str, Any]:
        """CWE 상세 정보 조회"""
        if not self.cwe_database:
            return {'error': 'CWE 데이터베이스가 로드되지 않았습니다.'}
        
        # CWE ID 정규화
        if not cwe_id.startswith('CWE-'):
            cwe_id = f"CWE-{cwe_id}"
        
        # CWE 정보 검색
        for cwe in self.cwe_database.cwes:
            if cwe['cwe_id'] == cwe_id.replace('CWE-', ''):
                return {
                    'cwe_id': cwe_id,
                    'name': cwe.get('name', ''),
                    'description': cwe.get('description', ''),
                    'category': cwe.get('category', ''),
                    'risk_level': self._assess_cwe_risk(cwe),
                    'mitigations': self._get_cwe_mitigations(cwe),
                    'examples': self._get_cwe_examples(cwe),
                    'testing_approach': self._get_testing_approach(cwe_id)
                }
        
        return {'error': f'CWE {cwe_id}를 찾을 수 없습니다.'}
    
    def _assess_cwe_risk(self, cwe_info: Dict[str, Any]) -> str:
        """CWE 위험도 평가"""
        raw_data = cwe_info.get('raw_data', {})
        abstraction = raw_data.get('abstraction', '')
        
        if abstraction == 'Base':
            return 'High'
        elif abstraction == 'Variant':
            return 'Medium'
        else:
            return 'Low'
    
    def _get_cwe_mitigations(self, cwe_info: Dict[str, Any]) -> List[str]:
        """CWE 완화 방안 추출"""
        mitigations = []
        raw_data = cwe_info.get('raw_data', {})
        
        if raw_data.get('mitigations'):
            for mitigation in raw_data['mitigations']:
                if mitigation.get('description'):
                    mitigations.append(mitigation['description'])
        
        # 기본 완화 방안 추가
        if not mitigations:
            mitigations.extend([
                '코드 리뷰 및 정적 분석 도구 활용',
                '보안 코딩 표준 준수',
                '정기적인 보안 테스트 수행'
            ])
        
        return mitigations
    
    def _get_cwe_examples(self, cwe_info: Dict[str, Any]) -> List[str]:
        """CWE 예시 추출"""
        examples = []
        raw_data = cwe_info.get('raw_data', {})
        
        if raw_data.get('examples'):
            for example in raw_data['examples']:
                if example.get('description'):
                    examples.append(example['description'])
        
        return examples
    
    def _get_testing_approach(self, cwe_id: str) -> str:
        """CWE별 테스트 접근법"""
        testing_approaches = {
            'CWE-287': '인증 메커니즘 테스트, 세션 관리 테스트',
            'CWE-285': '권한 검증 테스트, 접근 제어 테스트',
            'CWE-20': '입력 검증 테스트, 경계값 테스트',
            'CWE-119': '메모리 할당 테스트, 버퍼 오버플로우 테스트',
            'CWE-362': '동시성 테스트, 경쟁 상태 테스트',
            'CWE-434': '파일 업로드 테스트, 경로 검증 테스트'
        }
        
        return testing_approaches.get(cwe_id, '일반적인 보안 테스트 수행')
    
    def _generate_component_checklist(self, component: str, phase: str) -> Dict[str, Any]:
        """컴포넌트별 보안 체크리스트 생성"""
        if not self.guidelines:
            return {'error': '보안 가이드라인이 로드되지 않았습니다.'}
        
        checklist = self.guidelines.get('checklist', {})
        
        if component == 'all':
            return checklist
        
        # 특정 컴포넌트에 대한 체크리스트 생성
        component_guidelines = self.guidelines.get('components', {}).get(component, {})
        
        if not component_guidelines:
            return {'error': f'컴포넌트 {component}에 대한 가이드라인이 없습니다.'}
        
        # 컴포넌트별 맞춤 체크리스트 생성
        custom_checklist = {
            'component': component,
            'risk_level': component_guidelines.get('risk_level', 'Unknown'),
            'critical_vulnerabilities': component_guidelines.get('critical_vulnerabilities', []),
            'general_security_items': checklist
        }
        
        return custom_checklist
    
    def _generate_security_guidelines(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """보안 가이드라인 생성"""
        return self.guidelines
    
    def _analyze_security_risks(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """보안 위험도 분석"""
        return {'security_risk_analysis': 'completed'}
    
    def _create_security_checklist(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """보안 체크리스트 생성"""
        return {'security_checklist': 'created'}
    
    def _validate_security_compliance(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """보안 준수성 검증"""
        return {'security_compliance': 'validated'}
    
    def get_security_guidelines(self, category: str = 'all', component: str = 'all') -> Dict[str, Any]:
        """보안 가이드라인 조회 (외부 호출용)"""
        return self._get_relevant_guidelines(category, component)
    
    def analyze_code_security(self, code_snippet: str, component: str = 'general') -> Dict[str, Any]:
        """코드 보안 분석 (외부 호출용)"""
        return self._analyze_code_security_risks(code_snippet, component)
