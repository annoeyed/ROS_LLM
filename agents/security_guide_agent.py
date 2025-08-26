#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Security Guide Agent
CWE 데이터베이스를 기반으로 ROS 보안 가이드라인을 생성하고 제공하는 Agent
"""

import sys
import os
import json
import re
from typing import Dict, Any, List, Optional
from .base_agent import BaseAgent, AgentMessage, AgentTask

# Add parent directory path (for accessing rag_utils module)
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class SecurityGuideAgent(BaseAgent):
    """CWE 기반 보안 가이드라인 생성 및 제공 Agent"""
    
    def __init__(self, agent_id: str = "security_guide_001"):
        super().__init__(agent_id, "Security Guide Agent")
        self.llm_client = llm_client  # LLM 클라이언트는 BaseAgent에서 초기화됨
        # Initialize security guideline generator
        self.guidelines = None  # Initialize as None to set in _initialize
        self.cwe_database = None
        self.cwe_rag = None
        
        # Security risk levels
        self.risk_levels = {
            'critical': 4,
            'high': 3,
            'medium': 2,
            'low': 1
        }
    
    def _initialize(self):
        """Initialize Security Guide Agent"""
        super()._initialize()
        
        try:
            # Load CWE database and RAG system
            self._load_security_systems()
            self.logger.info("Security system loaded successfully")
        except Exception as e:
            self.logger.error(f"Failed to load security system: {e}")
            self.status = 'error'
    
    def _load_security_systems(self):
        """Load security systems (RAG integration)"""
        try:
            # Load environment variables
            try:
                from dotenv import load_dotenv
                import os
                # Set .env file path based on current file directory
                env_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env')
                load_dotenv(env_path)
            except ImportError:
                self.logger.warning("python-dotenv not installed. Failed to load environment variables")
            
            # Try to load CWE database
            try:
                from rag_utils.cwe_database import CWEDatabase
                self.cwe_database = CWEDatabase()
                self.cwe_database.load_database()
                self.logger.info("CWE database loaded successfully")
            except Exception as cwe_db_error:
                self.logger.warning(f"Failed to load CWE database: {cwe_db_error}")
                self.cwe_database = None
            
            # Load CWE RAG system (core functionality)
            try:
                from rag_utils.cwe_rag import CWERAGSearch
                # Set absolute path for CWE index directory
                cwe_index_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'rag_sources', 'cwe_index')
                self.cwe_rag = CWERAGSearch(cwe_index_dir)
                self.logger.info("CWE RAG system loaded successfully (core functionality)")
            except Exception as cwe_rag_error:
                self.logger.warning(f"Failed to load CWE RAG system: {cwe_rag_error}")
                self.cwe_rag = None
            
            # Generate RAG-based security guidelines
            if self.cwe_database and self.cwe_rag:
                try:
                    from rag_utils.security_guidelines import SecurityGuidelineGenerator
                    generator = SecurityGuidelineGenerator(self.cwe_database, self.cwe_rag)
                    self.guidelines = generator.generate_ros_security_guidelines()
                    self.logger.info(f"RAG-based security guidelines generated: {len(self.guidelines.get('categories', {}))} categories")
                    self.logger.info(f"Guideline content check: {type(self.guidelines)}, keys: {list(self.guidelines.keys()) if isinstance(self.guidelines, dict) else 'Not a dict'}")
                except Exception as guideline_error:
                    self.logger.warning(f"Failed to generate RAG-based guidelines: {guideline_error}")
                    self.logger.warning(f"Error details: {type(guideline_error)}, {str(guideline_error)}")
                    self.guidelines = self._create_enhanced_guidelines()
            else:
                # Generate enhanced basic guidelines when RAG system is not available
                self.guidelines = self._create_enhanced_guidelines()
                self.logger.info("Enhanced basic security guidelines generated")
            
        except Exception as e:
            self.logger.error(f"Critical error loading security systems: {e}")
            # 최종 대안으로 기본 가이드라인 생성
            self.guidelines = self._create_default_guidelines()
            self.logger.warning("기본 보안 가이드라인을 사용합니다.")
    
    def run(self, plan: str):
        """
        워크플로우에서 부르는 어댑터.
        Returns: (guidelines_markdown: str, rag_context: str, cwe_ids: List[str])
        """
        return self.generate_guildelines(plan)
    
    def generate_guildelines(self, plan: str):
        """
        주어진 plan에 대해 보안 가이드라인을 Markdown으로 만들어서 돌려준다.
        내부의 self.guidelines(dict) 를 RAG/기본 가이드라인 소스로 쓰고,
        llm_client가 있으면 요약/정리, 없으면 고정 Fallback 문구 사용.
        """
        #1) RAG/기본 가이드라인(dict)→ rag_context(JSON)
        baseline = self.get_security_guidelines(category='all', component='all')
        try:
            rag_context = json.dumps(baseline, ensure_ascii=False, indent=2)
        except Exception:
            rag_context = str(baseline)
        rag_context = self.truncate(rag_context, 8000)
        
        #2) Markdown 가이드라인 생성
        guidelines_md = None
        if getattr(self, 'llm_client', None):
            prompt = f"""
    You are a ROS 2 security expert. Create concise, actionable security guidelines
    for the plan below. Focus on preventing unauthorized runtime parameter changes
    (read-only parameters), safe node lifecycle, robust input validation, secret-safe
    logging, QoS consistency, and forbidding dangerous execution APIs.
    
    # plan
    {plan}
    
    # Baseline Guidelines (Context)
    {rag_context}
    
    # Write
    - One-sentence rationale.
- 8–12 concrete, verifiable bullets (imperative voice).
- Include: read-only parameters (no runtime mutation / disable dynamic reconfigure),
  try/except around init/spin/shutdown, least-privilege on params/services/topics,
  QoS consistency, secret masking in logs, and banning eval/exec/os.system/subprocess.
- End with: `CWE: CWE-xxx, ...`

Return Markdown only.
""".strip()
            try:
                guidelines_md = (self.llm_client.generate_response(
                    prompt,
                    temperature=float(os.getenv("AI_TEMPERATURE", "0.2")),
                    max_tokens=int(os.getenv("AI_MAX_TOKENS", "900")),
                ) or "").strip()
            except Exception as e:
                self.logger.warning(f"LLM call failed, using fallback guidelines: {e}")
                guidelines_md = self._fallback_guidelines()
        else:
            guidelines_md = self._fallback_guidelines()

        # 3) CWE ID 추출 (guidelines + rag_context 모두에서)
        cwe_ids = sorted(set(re.findall(r"CWE-\d+", f"{guidelines_md}\n{rag_context}")))
        return guidelines_md, rag_context, cwe_ids

    # (추가) 헬퍼들 ---------------------------------------------------------
    def _truncate(self, s: str, limit: int) -> str:
        if not isinstance(s, str):
            return ""
        return s if len(s) <= limit else (s[:limit] + "\n... (truncated)")

    def _fallback_guidelines(self) -> str:
        # LLM 없이도 항상 나오는 고정 Markdown
        return (
            "### Rationale\n"
            "Apply defense-in-depth for ROS 2 nodes. Prevent runtime parameter tampering and ensure safe lifecycle.\n\n"
            "### Rules\n"
            "- Declare sensitive parameters as **read-only** at startup; disable any dynamic reconfigure.\n"
            "- Validate all inputs (type/range/length); sanitize strings before using them in logic or I/O.\n"
            "- Wrap init/spin/shutdown in try/except; always destroy nodes and call `rclpy.shutdown()` in `finally`.\n"
            "- Mask secrets/PII in logs; never log tokens, keys, or credentials.\n"
            "- **Ban** `eval`, `exec`, `os.system`, and `subprocess.*` unless a strict allowlist is enforced.\n"
            "- Keep publisher/subscriber QoS profiles consistent and appropriate for reliability requirements.\n"
            "- Enforce least-privilege for services, topics, and parameters (RBAC).\n"
            "- Enable DDS Security (authentication, encryption, access control) where available.\n"
            "- Add tests that attempt to change read-only parameters and assert failure.\n\n"
            "CWE: CWE-284, CWE-306, CWE-20, CWE-200, CWE-78"
        ) 
    
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
        # 디버깅을 위한 로깅 추가
        self.logger.info(f"가이드라인 조회 요청: category={category}, component={component}")
        self.logger.info(f"현재 self.guidelines 상태: {type(self.guidelines)}, 내용: {self.guidelines}")
        
        if self.guidelines is None or (isinstance(self.guidelines, dict) and not self.guidelines):
            self.logger.warning("가이드라인이 비어있음. RAG 기반 가이드라인 재생성 시도...")
            # RAG 시스템 상태 확인
            self.logger.info(f"RAG 시스템 상태 확인: cwe_database={self.cwe_database is not None}, cwe_rag={self.cwe_rag is not None}")
            
            # RAG 시스템이 없으면 다시 초기화 시도
            if not self.cwe_database or not self.cwe_rag:
                self.logger.warning("RAG 시스템이 없음. 보안 시스템 재초기화 시도...")
                try:
                    self._load_security_systems()
                    self.logger.info("보안 시스템 재초기화 완료")
                except Exception as e:
                    self.logger.error(f"보안 시스템 재초기화 실패: {e}")
            
            # 가이드라인이 비어있으면 RAG 기반 가이드라인을 다시 생성 시도
            try:
                if self.cwe_database and self.cwe_rag:
                    from rag_utils.security_guidelines import SecurityGuidelineGenerator
                    generator = SecurityGuidelineGenerator(self.cwe_database, self.cwe_rag)
                    self.guidelines = generator.generate_ros_security_guidelines()
                    self.logger.info("RAG 기반 가이드라인 재생성 완료")
                else:
                    self.logger.warning("RAG 시스템이 여전히 없음. 향상된 기본 가이드라인 생성...")
                    self.guidelines = self._create_enhanced_guidelines()
                    self.logger.info("향상된 기본 가이드라인 생성 완료")
            except Exception as e:
                self.logger.error(f"가이드라인 재생성 실패: {e}")
                self.logger.warning("최종 대안으로 기본 가이드라인 생성...")
                self.guidelines = self._create_enhanced_guidelines()
        
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
        """코드 보안 분석 (외부 호출용) - RAG 통합"""
        return self._analyze_code_security_risks_with_rag(code_snippet, component)
    
    def _analyze_code_security_risks_with_rag(self, code_snippet: str, component: str) -> Dict[str, Any]:
        """RAG를 활용한 코드 보안 위험도 분석"""
        # 기본 패턴 기반 분석
        basic_analysis = self._analyze_code_security_risks(code_snippet, component)
        
        # RAG 기반 추가 분석
        rag_analysis = self._perform_rag_based_analysis(code_snippet, component)
        
        # 결과 통합
        combined_risks = basic_analysis.get('risks', [])
        combined_risks.extend(rag_analysis.get('rag_risks', []))
        
        # 위험도 점수 재계산
        total_risk_score = basic_analysis.get('risk_score', 0) + rag_analysis.get('rag_risk_score', 0)
        
        # 최종 위험도 레벨 결정
        if total_risk_score >= 20:
            final_risk_level = 'Critical'
        elif total_risk_score >= 15:
            final_risk_level = 'High'
        elif total_risk_score >= 10:
            final_risk_level = 'Medium'
        else:
            final_risk_level = 'Low'
        
        return {
            'risk_level': final_risk_level,
            'risk_score': total_risk_score,
            'risks': combined_risks,
            'basic_analysis': basic_analysis,
            'rag_analysis': rag_analysis,
            'recommendations': self._get_enhanced_security_recommendations(combined_risks, component),
            'rag_enhanced': True
        }
    
    def _perform_rag_based_analysis(self, code_snippet: str, component: str) -> Dict[str, Any]:
        """RAG 기반 보안 분석 수행"""
        if not self.cwe_rag:
            return {
                'rag_risks': [],
                'rag_risk_score': 0,
                'rag_enhanced': False,
                'message': 'RAG 시스템이 사용 불가능합니다.'
            }
        
        try:
            # RAG 검색 수행
            search_results = self.cwe_rag.search(code_snippet, top_k=5)
            
            rag_risks = []
            rag_risk_score = 0
            
            for result in search_results:
                if result.get('score', 0) > 0.7:  # 높은 유사도 결과만
                    cwe_info = result.get('metadata', {})
                    if cwe_info:
                        rag_risk = {
                            'type': 'rag_detected',
                            'title': f"RAG 검색 결과: {cwe_info.get('cwe_id', 'Unknown')}",
                            'description': cwe_info.get('description', ''),
                            'cwe_id': cwe_info.get('cwe_id', ''),
                            'severity_score': 8,  # RAG 검출 위험은 높은 점수
                            'source': 'CWE RAG',
                            'mitigation': cwe_info.get('mitigation', ''),
                            'confidence': result.get('score', 0)
                        }
                        rag_risks.append(rag_risk)
                        rag_risk_score += 8
            
            return {
                'rag_risks': rag_risks,
                'rag_risk_score': rag_risk_score,
                'rag_enhanced': True,
                'search_results_count': len(search_results),
                'high_confidence_results': len(rag_risks)
            }
            
        except Exception as e:
            self.logger.error(f"RAG 기반 분석 실패: {e}")
            return {
                'rag_risks': [],
                'rag_risk_score': 0,
                'rag_enhanced': False,
                'error': str(e)
            }
    
    def _get_enhanced_security_recommendations(self, risks: List[Dict], component: str) -> List[str]:
        """향상된 보안 권장사항 생성 (RAG 통합)"""
        recommendations = []
        
        # 기본 권장사항
        for risk in risks:
            if risk.get('mitigation'):
                recommendations.append(f"{risk['title']}: {risk['mitigation']}")
        
        # RAG 기반 추가 권장사항
        rag_recommendations = self._get_rag_based_recommendations(risks, component)
        recommendations.extend(rag_recommendations)
        
        # 컴포넌트별 추가 권장사항
        component_recommendations = self._get_component_specific_recommendations(component)
        recommendations.extend(component_recommendations)
        
        return list(set(recommendations))  # 중복 제거
    
    def _get_rag_based_recommendations(self, risks: List[Dict], component: str) -> List[str]:
        """RAG 기반 추가 권장사항 생성"""
        if not self.cwe_rag:
            return []
        
        recommendations = []
        
        try:
            # 위험 패턴을 RAG로 검색하여 추가 권장사항 생성
            for risk in risks:
                if risk.get('cwe_id'):
                    # CWE ID로 관련 정보 검색
                    cwe_search = f"CWE-{risk['cwe_id']}"
                    search_results = self.cwe_rag.search(cwe_search, top_k=3)
                    
                    for result in search_results:
                        if result.get('score', 0) > 0.8:
                            cwe_info = result.get('metadata', {})
                            if cwe_info.get('mitigation'):
                                recommendations.append(f"RAG 권장사항: {cwe_info['mitigation']}")
            
        except Exception as e:
            self.logger.error(f"RAG 기반 권장사항 생성 실패: {e}")
        
        return recommendations
    
    def _get_component_specific_recommendations(self, component: str) -> List[str]:
        """컴포넌트별 특화 권장사항"""
        component_recommendations = {
            'rclpy/rclcpp': [
                'ROS 2 보안 기능 활성화 (DDS 보안)',
                '토픽 암호화 설정',
                '노드 인증 및 권한 관리',
                '네임스페이스 격리 설정'
            ],
            'tf2': [
                '변환 데이터 검증 및 무결성 검사',
                '보안 토픽 사용',
                '메모리 사용량 모니터링'
            ],
            'urdf': [
                'XML 파싱 보안 설정 (XXE 방지)',
                '외부 파일 참조 검증',
                '파일 경로 검증'
            ],
            'gazebo': [
                '플러그인 보안 검증',
                '네트워크 접근 제한',
                '파일 시스템 접근 제한'
            ]
        }
        
        return component_recommendations.get(component, [])
    
    def _create_default_guidelines(self) -> Dict[str, Any]:
        """기본 보안 가이드라인 생성"""
        return {
            'categories': {
                'authentication': {
                    'title': '인증',
                    'description': '사용자 및 시스템 인증 관련 보안 가이드라인',
                    'guidelines': [
                        '강력한 비밀번호 정책 적용',
                        '다중 인증(MFA) 구현',
                        '세션 타임아웃 설정',
                        '인증 실패 시 계정 잠금'
                    ]
                },
                'authorization': {
                    'title': '권한 관리',
                    'description': '접근 제어 및 권한 관리 가이드라인',
                    'guidelines': [
                        '최소 권한 원칙 적용',
                        '역할 기반 접근 제어(RBAC) 구현',
                        '정기적인 권한 검토',
                        '권한 상승 방지'
                    ]
                },
                'input_validation': {
                    'title': '입력 검증',
                    'description': '사용자 입력 검증 및 필터링 가이드라인',
                    'guidelines': [
                        '모든 입력 데이터 검증',
                        'SQL 인젝션 방지',
                        'XSS 공격 방지',
                        '경로 순회 공격 방지'
                    ]
                },
                'memory_management': {
                    'title': '메모리 관리',
                    'description': '메모리 안전성 및 버퍼 오버플로우 방지',
                    'guidelines': [
                        '안전한 메모리 할당',
                        '버퍼 오버플로우 방지',
                        '메모리 누수 방지',
                        '포인터 검증'
                    ]
                },
                'file_operations': {
                    'title': '파일 작업',
                    'description': '파일 시스템 보안 가이드라인',
                    'guidelines': [
                        '파일 경로 검증',
                        '권한 확인',
                        '임시 파일 안전 처리',
                        '파일 업로드 검증'
                    ]
                }
            },
            'components': {
                'rclpy/rclcpp': {
                    'title': 'ROS 2 클라이언트 라이브러리',
                    'risk_level': 'Medium',
                    'guidelines': [
                        '노드 보안 설정',
                        '토픽 암호화',
                        '서비스 인증'
                    ]
                },
                'tf2': {
                    'title': 'Transform 라이브러리',
                    'risk_level': 'Low',
                    'guidelines': [
                        '변환 데이터 검증',
                        '보안 토픽 사용'
                    ]
                }
            },
            'checklist': {
                'general': [
                    '코드 리뷰 수행',
                    '정적 분석 도구 사용',
                    '보안 테스트 실행',
                    '의존성 취약점 검사'
                ]
            }
        }
    
    def _create_enhanced_guidelines(self) -> Dict[str, Any]:
        """향상된 기본 보안 가이드라인 생성 (AI 스타일)"""
        return {
            'categories': {
                'authentication': {
                    'title': '인증',
                    'description': '사용자 및 시스템 인증 관련 보안 가이드라인',
                    'guidelines': [
                        '강력한 비밀번호 정책 적용 (최소 12자, 특수문자 포함)',
                        '다중 인증(MFA) 구현 (TOTP, SMS, 하드웨어 토큰)',
                        '세션 타임아웃 설정 (15분 비활성 시 자동 로그아웃)',
                        '인증 실패 시 계정 잠금 (5회 실패 시 30분 잠금)',
                        'JWT 토큰 사용 시 짧은 만료 시간 설정',
                        'OAuth 2.0 및 OpenID Connect 구현'
                    ],
                    'ai_enhanced': True,
                    'risk_patterns': ['CWE-287', 'CWE-384', 'CWE-521']
                },
                'authorization': {
                    'title': '권한 관리',
                    'description': '접근 제어 및 권한 관리 가이드라인',
                    'guidelines': [
                        '최소 권한 원칙 적용 (필요한 권한만 부여)',
                        '역할 기반 접근 제어(RBAC) 구현',
                        '정기적인 권한 검토 (월 1회)',
                        '권한 상승 방지 (sudo 사용 제한)',
                        'API 엔드포인트별 권한 검증',
                        '세션 기반 권한 관리'
                    ],
                    'ai_enhanced': True,
                    'risk_patterns': ['CWE-285', 'CWE-862', 'CWE-269']
                },
                'input_validation': {
                    'title': '입력 검증',
                    'description': '사용자 입력 검증 및 필터링 가이드라인',
                    'guidelines': [
                        '모든 입력 데이터 검증 (화이트리스트 방식)',
                        'SQL 인젝션 방지 (매개변수화된 쿼리 사용)',
                        'XSS 공격 방지 (HTML 인코딩)',
                        '경로 순회 공격 방지 (경로 정규화)',
                        '파일 업로드 검증 (MIME 타입, 확장자)',
                        'JSON 스키마 검증'
                    ],
                    'ai_enhanced': True,
                    'risk_patterns': ['CWE-20', 'CWE-89', 'CWE-79', 'CWE-22']
                },
                'memory_management': {
                    'title': '메모리 관리',
                    'description': '메모리 안전성 및 버퍼 오버플로우 방지',
                    'guidelines': [
                        '안전한 메모리 할당 (malloc/free 검증)',
                        '버퍼 오버플로우 방지 (경계 검사)',
                        '메모리 누수 방지 (자동 메모리 관리)',
                        '포인터 검증 (NULL 포인터 체크)',
                        '스택 오버플로우 방지 (재귀 깊이 제한)',
                        '힙 오버플로우 방지 (할당 크기 검증)'
                    ],
                    'ai_enhanced': True,
                    'risk_patterns': ['CWE-119', 'CWE-125', 'CWE-787']
                },
                'file_operations': {
                    'title': '파일 작업',
                    'description': '파일 시스템 보안 가이드라인',
                    'guidelines': [
                        '파일 경로 검증 (절대 경로 사용 금지)',
                        '권한 확인 (읽기/쓰기 권한 검증)',
                        '임시 파일 안전 처리 (고유한 이름 생성)',
                        '파일 업로드 검증 (크기, 타입, 내용)',
                        '파일 다운로드 보안 (경로 순회 방지)',
                        '로그 파일 보안 (민감 정보 마스킹)'
                    ],
                    'ai_enhanced': True,
                    'risk_patterns': ['CWE-22', 'CWE-434', 'CWE-200']
                },
                'network_security': {
                    'title': '네트워크 보안',
                    'description': '네트워크 통신 보안 가이드라인',
                    'guidelines': [
                        'TLS/SSL 사용 (최소 TLS 1.2)',
                        '인증서 검증 (체인 검증)',
                        '포트 스캔 방지 (방화벽 설정)',
                        'DDoS 방어 (속도 제한)',
                        'VPN 사용 (암호화된 터널)',
                        '네트워크 모니터링 (침입 탐지)'
                    ],
                    'ai_enhanced': True,
                    'risk_patterns': ['CWE-200', 'CWE-295', 'CWE-400']
                },
                'cryptography': {
                    'title': '암호화',
                    'description': '암호화 및 해시 함수 보안 가이드라인',
                    'guidelines': [
                        '강력한 암호화 알고리즘 사용 (AES-256, RSA-2048)',
                        '안전한 해시 함수 사용 (SHA-256, bcrypt)',
                        '키 관리 (안전한 키 생성, 저장, 교체)',
                        '랜덤 수 생성 (암호학적으로 안전한)',
                        '암호화 모드 설정 (CBC, GCM)',
                        '키 길이 최소화 (AES: 128bit, RSA: 2048bit)'
                    ],
                    'ai_enhanced': True,
                    'risk_patterns': ['CWE-327', 'CWE-338', 'CWE-321']
                }
            },
            'components': {
                'rclpy/rclcpp': {
                    'title': 'ROS 2 클라이언트 라이브러리',
                    'risk_level': 'Medium',
                    'guidelines': [
                        '노드 보안 설정 (DDS 보안 활성화)',
                        '토픽 암호화 (TLS/DTLS 사용)',
                        '서비스 인증 (JWT 토큰)',
                        '네임스페이스 격리',
                        'QoS 설정 보안',
                        '로깅 보안 (민감 정보 마스킹)'
                    ],
                    'ai_enhanced': True,
                    'cwe_mappings': ['CWE-200', 'CWE-287', 'CWE-285']
                },
                'tf2': {
                    'title': 'Transform 라이브러리',
                    'risk_level': 'Low',
                    'guidelines': [
                        '변환 데이터 검증 (NaN, 무한대 값 체크)',
                        '보안 토픽 사용 (암호화된 통신)',
                        '변환 체인 무결성 검사',
                        '좌표계 검증',
                        '메모리 사용량 모니터링'
                    ],
                    'ai_enhanced': True,
                    'cwe_mappings': ['CWE-190', 'CWE-191']
                },
                'urdf': {
                    'title': 'URDF 파일',
                    'risk_level': 'Medium',
                    'guidelines': [
                        'XML 파싱 보안 설정 (XXE 방지)',
                        '외부 파일 참조 검증',
                        '파일 경로 검증',
                        '메모리 사용량 제한',
                        '로깅 보안'
                    ],
                    'ai_enhanced': True,
                    'cwe_mappings': ['CWE-611', 'CWE-22']
                },
                'gazebo': {
                    'title': 'Gazebo 시뮬레이터',
                    'risk_level': 'Medium',
                    'guidelines': [
                        '플러그인 보안 검증',
                        '네트워크 접근 제한',
                        '파일 시스템 접근 제한',
                        '메모리 사용량 모니터링',
                        '로깅 보안'
                    ],
                    'ai_enhanced': True,
                    'cwe_mappings': ['CWE-434', 'CWE-200']
                }
            },
            'checklist': {
                'general': [
                    '코드 리뷰 수행 (보안 전문가 참여)',
                    '정적 분석 도구 사용 (SonarQube, CodeQL)',
                    '보안 테스트 실행 (OWASP ZAP, Burp Suite)',
                    '의존성 취약점 검사 (npm audit, pip-audit)',
                    '침투 테스트 수행 (정기적)',
                    '보안 인시던트 대응 계획 수립'
                ],
                'development': [
                    '보안 코딩 표준 준수 (OWASP ASVS)',
                    '코드 서명 및 검증',
                    'CI/CD 파이프라인 보안',
                    '아티팩트 저장소 보안',
                    '개발 환경 격리'
                ],
                'deployment': [
                    '컨테이너 보안 (Docker 보안)',
                    '오케스트레이션 보안 (Kubernetes)',
                    '인프라 보안 (Terraform)',
                    '모니터링 및 로깅',
                    '백업 및 복구'
                ]
            },
            'ai_enhanced': True,
            'rag_integrated': True,
            'metadata': {
                'version': '2.0',
                'last_updated': '2024',
                'ai_generated': True,
                'security_level': 'high',
                'compliance': ['OWASP ASVS', 'NIST CSF', 'ISO 27001'],
                'rag_system': 'integrated'
            }
        }
    
    def _update_guidelines_with_feedback(self, feedback_details: str) -> Dict[str, Any]:
        """피드백을 받아 보안 가이드라인 수정"""
        try:
            # 피드백 내용을 분석하여 가이드라인 업데이트
            updated_guidelines = self.guidelines.copy()
            
            # 피드백에 따른 새로운 가이드라인 추가
            if 'safety' in feedback_details.lower():
                if 'safety_guidelines' not in updated_guidelines:
                    updated_guidelines['safety_guidelines'] = []
                
                updated_guidelines['safety_guidelines'].append({
                    'feedback': feedback_details,
                    'timestamp': '2024',
                    'priority': 'high'
                })
            
            return {
                'status': 'success',
                'updated_guidelines': updated_guidelines,
                'feedback_applied': True
            }
            
        except Exception as e:
            self.logger.error(f"가이드라인 업데이트 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e)
            }
    
    def _rag_based_security_verification(self, algorithm_description: str, component: str) -> Dict[str, Any]:
        """RAG 기반 보안 검증 (RAG Guard Agent 기능 통합)"""
        try:
            if not self.cwe_rag:
                return {
                    'status': 'failed',
                    'error': 'RAG 시스템이 사용 불가능합니다.',
                    'rag_enhanced': False
                }
            
            # RAG 검색을 통한 보안 검증
            search_results = self.cwe_rag.search(algorithm_description, top_k=5)
            
            security_issues = []
            total_risk_score = 0
            
            for result in search_results:
                if result.score > 0.7:  # 높은 유사도 결과만
                    chunk = result.chunk
                    # CWE ID 추출 (제목에서 CWE-XXX 형식 찾기)
                    cwe_id = "Unknown"
                    if "CWE-" in chunk.title:
                        cwe_id = chunk.title.split("CWE-")[1].split(":")[0]
                    
                    security_issue = {
                        'type': 'rag_detected',
                        'title': f"RAG 검색 결과: CWE-{cwe_id}",
                        'description': chunk.text[:200] + "..." if len(chunk.text) > 200 else chunk.text,
                        'cwe_id': f"CWE-{cwe_id}",
                        'severity_score': 10,
                        'source': 'CWE RAG',
                        'mitigation': '코드 리뷰 및 보안 테스트 수행',
                        'confidence': result.score
                    }
                    security_issues.append(security_issue)
                    total_risk_score += 10
            
            # 보안 위험도 레벨 결정
            if total_risk_score >= 30:
                risk_level = 'Critical'
            elif total_risk_score >= 20:
                risk_level = 'High'
            elif total_risk_score >= 10:
                risk_level = 'Medium'
            else:
                risk_level = 'Low'
            
            return {
                'status': 'completed',
                'rag_enhanced': True,
                'security_issues': security_issues,
                'total_risk_score': total_risk_score,
                'risk_level': risk_level,
                'search_results_count': len(search_results),
                'high_confidence_issues': len(security_issues),
                'recommendations': self._generate_security_recommendations_from_rag(security_issues, component)
            }
            
        except Exception as e:
            self.logger.error(f"RAG 기반 보안 검증 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e),
                'rag_enhanced': False
            }
    
    def _generate_security_recommendations_from_rag(self, security_issues: List[Dict], component: str) -> List[str]:
        """RAG 검색 결과로부터 보안 권장사항 생성"""
        recommendations = []
        
        for issue in security_issues:
            if issue.get('mitigation'):
                recommendations.append(f"{issue['title']}: {issue['mitigation']}")
        
        # 컴포넌트별 추가 권장사항
        component_specific = self._get_component_specific_recommendations(component)
        recommendations.extend(component_specific)
        
        return list(set(recommendations))  # 중복 제거
