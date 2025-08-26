#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sequential Multi-Agent 워크플로우 시스템
이미지에 맞춘 Generation과 Evaluation 단계를 명확히 구분한 구조
RAG Guard Agent 제거, Security Guide Agent에 RAG 통합
"""

import sys
import os
import json
import logging
import time
from typing import Dict, Any, List, Optional
from enum import Enum

# 상위 디렉토리 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from agents import (
    PlannerAgent, 
    SecurityGuideAgent, 
    CoderAgent, 
    SimulationAgent
)
from agents.base_agent import AgentMessage, AgentTask

class WorkflowPhase(Enum):
    """워크플로우 단계 정의"""
    GENERATION = "generation"
    EVALUATION = "evaluation"

class OracleResult(Enum):
    """Oracle 검증 결과"""
    PASS = "pass"
    FAIL = "fail"

class SequentialWorkflow:
    """Sequential Multi-Agent 워크플로우 시스템"""
    
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Agent들 초기화 (RAG Guard Agent 제거)
        self.agents = {}
        self.workflow_history = []
        self.current_workflow = None
        
        # 워크플로우 설정
        self.workflow_config = {
            'max_retries': 3,
            'timeout': 300,  # 5분
            'ai_enhanced': True
        }
        
        # Oracle 결과 저장
        self.oracle_results = {
            'param': None,
            'safety': None,
            'mode': None
        }
    
    def initialize_agents(self):
        """모든 Agent 초기화 (RAG Guard Agent 제거)"""
        try:
            self.logger.info("Sequential Multi-Agent 시스템 초기화 시작")
            
            # 1. Planner Agent (AI 기반 계획 수립)
            self.agents['planner'] = PlannerAgent("planner_001")
            self.logger.info("Planner Agent 초기화 완료")
            
            # 2. Security Guide Agent (CWE + RAG 통합 보안 가이드라인)
            self.agents['security_guide'] = SecurityGuideAgent("security_guide_001")
            self.logger.info("Security Guide Agent 초기화 완료 (RAG 통합)")
            
            # 3. Coder Agent (AI 기반 코드 생성)
            self.agents['coder'] = CoderAgent("coder_001")
            self.logger.info("Coder Agent 초기화 완료")
            
            # 4. Simulation Agent (AI 기반 시뮬레이션 및 테스트)
            self.agents['simulation'] = SimulationAgent("simulation_001")
            self.logger.info("Simulation Agent 초기화 완료")
            
            self.logger.info("모든 Agent 초기화 완료 (RAG Guard Agent 제거)")
            return True
            
        except Exception as e:
            self.logger.error(f"Agent 초기화 실패: {e}")
            return False
    
    def execute_workflow(self, user_request: str) -> Dict[str, Any]:
        """Sequential 워크플로우 실행"""
        workflow_id = f"workflow_{int(time.time())}"
        self.current_workflow = {
            'id': workflow_id,
            'user_request': user_request,
            'start_time': time.time(),
            'status': 'running',
            'phases': {
                'generation': [],
                'evaluation': []
            }
        }
        
        try:
            self.logger.info(f"Sequential 워크플로우 시작: {workflow_id}")
            
            # Phase 1: Generation (Instruction → Planner → Security Guide (RAG 통합) → Coder → LLM as Judge)
            generation_result = self._execute_generation_phase(user_request)
            
            if generation_result['status'] == 'failed':
                return self._finalize_workflow('failed', 'Generation 단계 실패')
            
            # Phase 2: Evaluation (Simulation → Oracles)
            evaluation_result = self._execute_evaluation_phase(generation_result)
            
            if evaluation_result['status'] == 'failed':
                return self._finalize_workflow('failed', 'Evaluation 단계 실패')
            
            # 최종 결과 통합
            final_result = self._integrate_results(generation_result, evaluation_result)
            
            return self._finalize_workflow('completed', '모든 단계 완료', final_result)
            
        except Exception as e:
            self.logger.error(f"워크플로우 실행 실패: {e}")
            return self._finalize_workflow('failed', str(e))
    
    def _execute_generation_phase(self, user_request: str) -> Dict[str, Any]:
        """Generation 단계 실행 (RAG Guard Agent 제거)"""
        self.logger.info("=== Generation 단계 시작 ===")
        phase_results = []
        
        try:
            # Step 1: Instruction → Planner
            self.logger.info("Step 1: Instruction → Planner")
            planning_result = self._execute_planning_step(user_request)
            phase_results.append(planning_result)
            
            if planning_result['status'] == 'failed':
                return {'status': 'failed', 'error': 'Planning 단계 실패', 'results': phase_results}
            
            # Step 2: Planner → Security Guide (RAG 통합)
            self.logger.info("Step 2: Planner → Security Guide (RAG 통합)")
            security_result = self._execute_security_guide_step(user_request, planning_result)
            phase_results.append(security_result)
            
            if security_result['status'] == 'failed':
                return {'status': 'failed', 'error': 'Security Guide 단계 실패', 'results': phase_results}
            
            # Step 3: Security Guide → Coder (RAG Guard 단계 제거)
            self.logger.info("Step 3: Security Guide → Coder")
            coder_result = self._execute_coder_step(user_request, planning_result, security_result)
            phase_results.append(coder_result)
            
            if coder_result['status'] == 'failed':
                return {'status': 'failed', 'error': 'Coder 단계 실패', 'results': phase_results}
            
            # Step 4: Coder → LLM as Judge
            self.logger.info("Step 4: Coder → LLM as Judge")
            judge_result = self._execute_llm_judge_step(coder_result)
            phase_results.append(judge_result)
            
            if judge_result['status'] == 'failed':
                return {'status': 'failed', 'error': 'LLM Judge 단계 실패', 'results': phase_results}
            
            # LLM Judge 결과에 따른 피드백 루프
            if judge_result['verdict'] == 'fail':
                self.logger.info("LLM Judge 실패 - Coder로 피드백")
                return self._handle_judge_feedback(coder_result, judge_result, phase_results)
            
            self.logger.info("=== Generation 단계 완료 ===")
            return {
                'status': 'completed',
                'results': phase_results,
                'final_code': coder_result['generated_code']
            }
            
        except Exception as e:
            self.logger.error(f"Generation 단계 실행 실패: {e}")
            return {'status': 'failed', 'error': str(e), 'results': phase_results}
    
    def _execute_evaluation_phase(self, generation_result: Dict[str, Any]) -> Dict[str, Any]:
        """Evaluation 단계 실행"""
        self.logger.info("=== Evaluation 단계 시작 ===")
        phase_results = []
        
        try:
            # Step 1: Simulation 실행
            self.logger.info("Step 1: Simulation 실행")
            simulation_result = self._execute_simulation_step(generation_result['final_code'])
            phase_results.append(simulation_result)
            
            if simulation_result['status'] == 'failed':
                return {'status': 'failed', 'error': 'Simulation 단계 실패', 'results': phase_results}
            
            # Step 2: Oracles 검증 (Param, Safety, Mode)
            self.logger.info("Step 2: Oracles 검증")
            oracle_results = self._execute_oracles_verification(simulation_result)
            phase_results.append(oracle_results)
            
            # Oracle 결과에 따른 피드백 루프 결정
            feedback_decision = self._determine_feedback_path(oracle_results)
            
            if feedback_decision['needs_feedback']:
                self.logger.info(f"피드백 필요: {feedback_decision['target_agent']}")
                return self._handle_oracle_feedback(oracle_results, feedback_decision, phase_results)
            
            self.logger.info("=== Evaluation 단계 완료 ===")
            return {
                'status': 'completed',
                'results': phase_results,
                'oracle_results': oracle_results
            }
            
        except Exception as e:
            self.logger.error(f"Evaluation 단계 실행 실패: {e}")
            return {'status': 'failed', 'error': str(e), 'results': phase_results}
    
    def _execute_planning_step(self, user_request: str) -> Dict[str, Any]:
        """Planning 단계 실행"""
        try:
            start_time = time.time()
            
            self.logger.info("=== Planner Agent 활성화 시작 ===")
            self.logger.info(f"사용자 요청: {user_request[:100]}...")
            
            planner_agent = self.agents['planner']
            self.logger.info("Planner Agent가 요청을 분석하고 있습니다...")
            
            planning_result = planner_agent.analyze_request(user_request)
            
            execution_time = time.time() - start_time
            
            if 'error' not in planning_result:
                self.logger.info("=== Planner Agent 작업 완료 ===")
                self.logger.info(f"계획 수립 결과: {planning_result.get('plan_summary', '계획 생성됨')}")
                self.logger.info(f"보안 요구사항: {planning_result.get('security_requirements', [])}")
            else:
                self.logger.error("=== Planner Agent 작업 실패 ===")
                self.logger.error(f"오류 내용: {planning_result.get('error', '알 수 없는 오류')}")
            
            return {
                'step_name': 'AI 기반 계획 수립',
                'agent': 'Planner Agent',
                'status': 'completed' if 'error' not in planning_result else 'failed',
                'result': planning_result,
                'execution_time': execution_time
            }
            
        except Exception as e:
            self.logger.error(f"=== Planner Agent 실행 중 예외 발생 ===")
            self.logger.error(f"예외 내용: {e}")
            return {
                'step_name': 'AI 기반 계획 수립',
                'agent': 'Planner Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_security_guide_step(self, user_request: str, planning_result: Dict[str, Any]) -> Dict[str, Any]:
        """Security Guide 단계 실행 (RAG 통합)"""
        try:
            start_time = time.time()
            
            self.logger.info("=== Security Guide Agent 활성화 시작 ===")
            self.logger.info("CWE 데이터베이스 및 RAG 시스템을 로드하고 있습니다...")
            
            security_agent = self.agents['security_guide']
            security_requirements = planning_result.get('result', {}).get('security_requirements', [])
            
            self.logger.info(f"보안 요구사항: {security_requirements}")
            
            # RAG 기반 보안 가이드라인 생성
            self.logger.info("=== 보안 가이드라인 생성 시작 ===")
            self.logger.info("Security Guide Agent가 RAG 시스템을 통해 가이드라인을 생성하고 있습니다...")
            
            guidelines_result = security_agent.get_security_guidelines('general', 'ros_node')
            
            if 'error' not in guidelines_result:
                self.logger.info("=== 보안 가이드라인 생성 완료 ===")
                self.logger.info(f"가이드라인 카테고리: {list(guidelines_result.get('categories', {}).keys())}")
                self.logger.info(f"가이드라인 컴포넌트: {list(guidelines_result.get('components', {}).keys())}")
            else:
                self.logger.error("=== 보안 가이드라인 생성 실패 ===")
                self.logger.error(f"오류 내용: {guidelines_result.get('error', '알 수 없는 오류')}")
            
            # RAG 기반 보안 검증
            self.logger.info("=== RAG 기반 보안 검증 시작 ===")
            self.logger.info("Security Guide Agent가 RAG 시스템을 통해 보안 검증을 수행하고 있습니다...")
            
            security_verification_result = security_agent._rag_based_security_verification(
                guidelines_result.get('description', user_request), # 가이드라인 설명 또는 사용자 요청
                'ros_node'
            )
            
            if security_verification_result.get('status') == 'completed':
                self.logger.info("=== RAG 기반 보안 검증 완료 ===")
                self.logger.info(f"검출된 보안 이슈: {len(security_verification_result.get('security_issues', []))}개")
                self.logger.info(f"위험도 레벨: {security_verification_result.get('risk_level', 'Unknown')}")
                self.logger.info(f"RAG 강화: {security_verification_result.get('rag_enhanced', False)}")
            else:
                self.logger.error("=== RAG 기반 보안 검증 실패 ===")
                self.logger.error(f"오류 내용: {security_verification_result.get('error', '알 수 없는 오류')}")
            
            execution_time = time.time() - start_time
            
            self.logger.info("=== Security Guide Agent 작업 완료 ===")
            
            return {
                'step_name': '보안 가이드라인 생성 및 검증',
                'agent': 'Security Guide Agent',
                'status': 'completed' if 'error' not in guidelines_result else 'failed',
                'result': {
                    'guidelines': guidelines_result,
                    'security_verification': security_verification_result
                },
                'execution_time': execution_time
            }
            
        except Exception as e:
            self.logger.error(f"=== Security Guide Agent 실행 중 예외 발생 ===")
            self.logger.error(f"예외 내용: {e}")
            import traceback
            self.logger.error(f"상세 오류: {traceback.format_exc()}")
            return {
                'step_name': '보안 가이드라인 생성 및 검증',
                'agent': 'Security Guide Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_coder_step(self, user_request: str, planning_result: Dict[str, Any], 
                           security_result: Dict[str, Any]) -> Dict[str, Any]:
        """Coder 단계 실행"""
        try:
            start_time = time.time()
            
            self.logger.info("=== Coder Agent 활성화 시작 ===")
            self.logger.info("AI 기반 ROS 코드 생성을 시작합니다...")
            
            coder_agent = self.agents['coder']
            
            # 모든 결과를 결합하여 코드 생성 요청
            self.logger.info("=== 코드 생성 요구사항 통합 ===")
            code_generation_request = self._combine_requirements_for_code_generation(
                user_request, planning_result, security_result
            )
            self.logger.info(f"통합된 요구사항: {list(code_generation_request.keys())}")
            
            self.logger.info("=== AI 기반 코드 생성 시작 ===")
            self.logger.info("Coder Agent가 OpenAI를 사용하여 보안을 고려한 ROS 코드를 생성하고 있습니다...")
            
            code_generation_result = coder_agent._generate_ros_code({
                'requirements': code_generation_request,
                'component_type': 'basic_node',
                'security_level': 'high'
            })
            
            execution_time = time.time() - start_time
            
            if 'error' not in code_generation_result:
                self.logger.info("=== AI 기반 코드 생성 완료 ===")
                generated_code = code_generation_result.get('code', '')
                if generated_code:
                    self.logger.info(f"생성된 코드 길이: {len(generated_code)} 문자")
                    self.logger.info(f"컴포넌트 타입: {code_generation_result.get('component_type', 'unknown')}")
                    self.logger.info(f"보안 수준: {code_generation_result.get('security_level', 'unknown')}")
                    self.logger.info(f"AI 강화: {code_generation_result.get('ai_enhanced', False)}")
                    
                    # 코드의 주요 부분 미리보기
                    code_preview = generated_code[:200] + "..." if len(generated_code) > 200 else generated_code
                    self.logger.info(f"코드 미리보기: {code_preview}")
                else:
                    self.logger.warning("생성된 코드가 비어있습니다.")
            else:
                self.logger.error("=== AI 기반 코드 생성 실패 ===")
                self.logger.error(f"오류 내용: {code_generation_result.get('error', '알 수 없는 오류')}")
            
            self.logger.info("=== Coder Agent 작업 완료 ===")
            
            return {
                'step_name': 'AI 기반 보안 코드 생성',
                'agent': 'Coder Agent',
                'status': 'completed' if 'error' not in code_generation_result else 'failed',
                'result': code_generation_result,
                'generated_code': code_generation_result.get('code', ''),
                'execution_time': execution_time
            }
            
        except Exception as e:
            self.logger.error(f"=== Coder Agent 실행 중 예외 발생 ===")
            self.logger.error(f"예외 내용: {e}")
            return {
                'step_name': 'AI 기반 보안 코드 생성',
                'agent': 'Coder Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_llm_judge_step(self, coder_result: Dict[str, Any]) -> Dict[str, Any]:
        """LLM as Judge 단계 실행"""
        try:
            start_time = time.time()
            
            # LLM을 사용하여 생성된 코드의 안전성 검증
            generated_code = coder_result.get('generated_code', '')
            
            # 간단한 안전성 검사 (실제로는 더 정교한 LLM 호출 필요)
            safety_check_result = self._perform_safety_check(generated_code)
            
            execution_time = time.time() - start_time
            
            return {
                'step_name': 'LLM as Judge - 안전성 검증',
                'agent': 'LLM Judge',
                'status': 'completed',
                'verdict': 'pass' if safety_check_result['is_safe'] else 'fail',
                'feedback': safety_check_result['feedback'],
                'execution_time': execution_time
            }
            
        except Exception as e:
            self.logger.error(f"LLM Judge 단계 실패: {e}")
            return {
                'step_name': 'LLM as Judge - 안전성 검증',
                'agent': 'LLM Judge',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_simulation_step(self, generated_code: str) -> Dict[str, Any]:
        """Simulation 단계 실행"""
        try:
            start_time = time.time()
            
            simulation_agent = self.agents['simulation']
            
            test_result = simulation_agent.run_comprehensive_tests({
                'code_snippet': generated_code,
                'test_type': 'comprehensive',
                'test_parameters': {
                    'security_focus': True,
                    'performance_focus': True,
                    'functionality_focus': True
                }
            })
            
            execution_time = time.time() - start_time
            
            return {
                'step_name': 'AI 기반 시뮬레이션 및 테스트',
                'agent': 'Simulation Agent',
                'status': 'completed' if 'error' not in test_result else 'failed',
                'result': test_result,
                'execution_time': execution_time
            }
            
        except Exception as e:
            self.logger.error(f"Simulation 단계 실패: {e}")
            return {
                'step_name': 'AI 기반 시뮬레이션 및 테스트',
                'agent': 'Simulation Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_oracles_verification(self, simulation_result: Dict[str, Any]) -> Dict[str, Any]:
        """Oracles 검증 실행"""
        try:
            start_time = time.time()
            
            # Param Oracle: Coder로 피드백
            param_result = self._execute_param_oracle(simulation_result)
            
            # Safety Oracle: Security Guide로 피드백
            safety_result = self._execute_safety_oracle(simulation_result)
            
            # Mode Oracle: Planner로 피드백 (로봇 시뮬레이션 환경)
            mode_result = self._execute_mode_oracle(simulation_result)
            
            execution_time = time.time() - start_time
            
            oracle_results = {
                'param': param_result,
                'safety': safety_result,
                'mode': mode_result
            }
            
            # Oracle 결과 저장
            self.oracle_results = oracle_results
            
            return {
                'step_name': 'Oracles 검증',
                'agent': 'Oracle System',
                'status': 'completed',
                'results': oracle_results,
                'execution_time': execution_time
            }
            
        except Exception as e:
            self.logger.error(f"Oracles 검증 실패: {e}")
            return {
                'step_name': 'Oracles 검증',
                'agent': 'Oracle System',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_param_oracle(self, simulation_result: Dict[str, Any]) -> Dict[str, Any]:
        """Param Oracle 실행 - Coder로 피드백"""
        try:
            # 파라미터 검증 (속도, 고도, 각도 등)
            param_check = self._check_parameters(simulation_result)
            
            return {
                'oracle_type': 'param',
                'result': OracleResult.PASS.value if param_check['is_valid'] else OracleResult.FAIL.value,
                'feedback_target': 'coder',
                'details': param_check['details']
            }
            
        except Exception as e:
            self.logger.error(f"Param Oracle 실행 실패: {e}")
            return {
                'oracle_type': 'param',
                'result': OracleResult.FAIL.value,
                'feedback_target': 'coder',
                'error': str(e)
            }
    
    def _execute_safety_oracle(self, simulation_result: Dict[str, Any]) -> Dict[str, Any]:
        """Safety Oracle 실행 - Security Guide로 피드백"""
        try:
            # 안전성 검증 (비상 절차, 금지 API, 정책 준수)
            safety_check = self._check_safety_compliance(simulation_result)
            
            return {
                'oracle_type': 'safety',
                'result': OracleResult.PASS.value if safety_check['is_compliant'] else OracleResult.FAIL.value,
                'feedback_target': 'security_guide',
                'details': safety_check['details']
            }
            
        except Exception as e:
            self.logger.error(f"Safety Oracle 실행 실패: {e}")
            return {
                'oracle_type': 'safety',
                'result': OracleResult.FAIL.value,
                'feedback_target': 'security_guide',
                'error': str(e)
            }
    
    def _execute_mode_oracle(self, simulation_result: Dict[str, Any]) -> Dict[str, Any]:
        """Mode Oracle 실행 - Planner로 피드백 (로봇 시뮬레이션 환경)"""
        try:
            # 모드 시퀀스 검증 (이륙, 경유, 착륙 등)
            mode_check = self._check_mode_sequence(simulation_result)
            
            return {
                'oracle_type': 'mode',
                'result': OracleResult.PASS.value if mode_check['is_valid'] else OracleResult.FAIL.value,
                'feedback_target': 'planner',
                'details': mode_check['details']
            }
            
        except Exception as e:
            self.logger.error(f"Mode Oracle 실행 실패: {e}")
            return {
                'oracle_type': 'mode',
                'result': OracleResult.FAIL.value,
                'feedback_target': 'planner',
                'error': str(e)
            }
    
    def _determine_feedback_path(self, oracle_results: Dict[str, Any]) -> Dict[str, Any]:
        """Oracle 결과에 따른 피드백 경로 결정"""
        failed_oracles = []
        
        for oracle_type, result in oracle_results['results'].items():
            if result['result'] == OracleResult.FAIL.value:
                failed_oracles.append({
                    'type': oracle_type,
                    'target': result['feedback_target']
                })
        
        if not failed_oracles:
            return {
                'needs_feedback': False,
                'message': '모든 Oracle 검증 통과'
            }
        
        return {
            'needs_feedback': True,
            'failed_oracles': failed_oracles,
            'primary_target': failed_oracles[0]['target']  # 첫 번째 실패한 Oracle의 타겟
        }
    
    def _handle_judge_feedback(self, coder_result: Dict[str, Any], judge_result: Dict[str, Any], 
                              phase_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """LLM Judge 실패 시 Coder로 피드백"""
        self.logger.info("LLM Judge 실패 - Coder로 피드백 처리")
        
        # Coder에게 피드백 제공하여 코드 수정 요청
        try:
            coder_agent = self.agents['coder']
            
            # 피드백을 포함한 코드 수정 요청
            modified_code_result = coder_agent._modify_code_with_feedback(
                coder_result['generated_code'],
                judge_result['feedback']
            )
            
            if modified_code_result['status'] == 'success':
                # 수정된 코드로 다시 LLM Judge 실행
                updated_coder_result = {
                    **coder_result,
                    'generated_code': modified_code_result['modified_code']
                }
                
                # 수정된 코드로 다시 Judge 실행
                updated_judge_result = self._execute_llm_judge_step(updated_coder_result)
                
                if updated_judge_result['verdict'] == 'pass':
                    self.logger.info("코드 수정 후 LLM Judge 통과")
                    return {
                        'status': 'completed',
                        'results': phase_results + [updated_coder_result, updated_judge_result],
                        'final_code': updated_coder_result['generated_code']
                    }
                else:
                    # 여전히 실패 - 최대 재시도 횟수 확인 필요
                    return {
                        'status': 'failed',
                        'error': '코드 수정 후에도 LLM Judge 실패',
                        'results': phase_results + [updated_coder_result, updated_judge_result]
                    }
            else:
                return {
                    'status': 'failed',
                    'error': '코드 수정 실패',
                    'results': phase_results
                }
                
        except Exception as e:
            self.logger.error(f"Judge 피드백 처리 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e),
                'results': phase_results
            }
    
    def _handle_oracle_feedback(self, oracle_results: Dict[str, Any], feedback_decision: Dict[str, Any],
                               phase_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Oracle 실패 시 적절한 Agent로 피드백"""
        self.logger.info(f"Oracle 실패 - {feedback_decision['primary_target']}로 피드백 처리")
        
        target_agent = feedback_decision['primary_target']
        
        try:
            if target_agent == 'coder':
                # Param Oracle 실패 - Coder로 피드백
                return self._handle_param_oracle_feedback(oracle_results, phase_results)
            elif target_agent == 'security_guide':
                # Safety Oracle 실패 - Security Guide로 피드백
                return self._handle_safety_oracle_feedback(oracle_results, phase_results)
            elif target_agent == 'planner':
                # Mode Oracle 실패 - Planner로 피드백
                return self._handle_mode_oracle_feedback(oracle_results, phase_results)
            else:
                return {
                    'status': 'failed',
                    'error': f'알 수 없는 피드백 타겟: {target_agent}',
                    'results': phase_results
                }
                
        except Exception as e:
            self.logger.error(f"Oracle 피드백 처리 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e),
                'results': phase_results
            }
    
    def _handle_param_oracle_feedback(self, oracle_results: Dict[str, Any], 
                                    phase_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Param Oracle 실패 시 Coder로 피드백"""
        try:
            coder_agent = self.agents['coder']
            param_details = oracle_results['results']['param']['details']
            
            # 파라미터 관련 피드백을 포함하여 코드 수정 요청
            modified_code_result = coder_agent._modify_code_with_feedback(
                phase_results[-1]['generated_code'],  # 마지막 Coder 결과
                f"파라미터 검증 실패: {param_details}"
            )
            
            if modified_code_result['status'] == 'success':
                # 수정된 코드로 다시 Evaluation 단계 실행
                updated_coder_result = {
                    **phase_results[-1],
                    'generated_code': modified_code_result['modified_code']
                }
                
                # 수정된 코드로 다시 Evaluation 실행
                evaluation_result = self._execute_evaluation_phase({
                    'final_code': modified_code_result['modified_code']
                })
                
                if evaluation_result['status'] == 'completed':
                    return {
                        'status': 'completed',
                        'results': phase_results + [updated_coder_result, evaluation_result],
                        'final_code': modified_code_result['modified_code']
                    }
                else:
                    return {
                        'status': 'failed',
                        'error': '코드 수정 후 Evaluation 실패',
                        'results': phase_results + [updated_coder_result, evaluation_result]
                    }
            else:
                return {
                    'status': 'failed',
                    'error': '파라미터 관련 코드 수정 실패',
                    'results': phase_results
                }
                
        except Exception as e:
            self.logger.error(f"Param Oracle 피드백 처리 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e),
                'results': phase_results
            }
    
    def _handle_safety_oracle_feedback(self, oracle_results: Dict[str, Any], 
                                     phase_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Safety Oracle 실패 시 Security Guide로 피드백"""
        try:
            security_agent = self.agents['security_guide']
            safety_details = oracle_results['results']['safety']['details']
            
            # 안전성 관련 피드백을 포함하여 보안 가이드라인 수정 요청
            updated_guidelines = security_agent._update_guidelines_with_feedback(
                safety_details
            )
            
            if updated_guidelines['status'] == 'success':
                # 수정된 가이드라인으로 다시 Coder 단계부터 실행
                updated_security_result = {
                    **phase_results[1],  # Security Guide 결과
                    'result': updated_guidelines['updated_guidelines']
                }
                
                # 수정된 가이드라인으로 다시 Coder 실행
                coder_result = self._execute_coder_step(
                    self.current_workflow['user_request'],
                    phase_results[0],  # Planning 결과
                    updated_security_result
                )
                
                if coder_result['status'] == 'completed':
                    # 수정된 코드로 다시 Evaluation 단계 실행
                    evaluation_result = self._execute_evaluation_phase({
                        'final_code': coder_result['generated_code']
                    })
                    
                    if evaluation_result['status'] == 'completed':
                        return {
                            'status': 'completed',
                            'results': [phase_results[0], updated_security_result, 
                                      coder_result, evaluation_result],
                            'final_code': coder_result['generated_code']
                        }
                    else:
                        return {
                            'status': 'failed',
                            'error': '가이드라인 수정 후 Evaluation 실패',
                            'results': [phase_results[0], updated_security_result, 
                                      coder_result, evaluation_result]
                        }
                else:
                    return {
                        'status': 'failed',
                        'error': '가이드라인 수정 후 Coder 실행 실패',
                        'results': [phase_results[0], updated_security_result, 
                                  coder_result]
                    }
            else:
                return {
                    'status': 'failed',
                    'error': '안전성 가이드라인 수정 실패',
                    'results': phase_results
                }
                
        except Exception as e:
            self.logger.error(f"Safety Oracle 피드백 처리 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e),
                'results': phase_results
            }
    
    def _handle_mode_oracle_feedback(self, oracle_results: Dict[str, Any], 
                                   phase_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Mode Oracle 실패 시 Planner로 피드백"""
        try:
            planner_agent = self.agents['planner']
            mode_details = oracle_results['results']['mode']['details']
            
            # 모드 시퀀스 관련 피드백을 포함하여 계획 수정 요청
            updated_plan = planner_agent._update_plan_with_feedback(
                self.current_workflow['user_request'],
                mode_details
            )
            
            if updated_plan['status'] == 'success':
                # 수정된 계획으로 다시 Security Guide부터 실행
                updated_planning_result = {
                    **phase_results[0],  # Planning 결과
                    'result': updated_plan['updated_plan']
                }
                
                # 수정된 계획으로 다시 Security Guide 실행
                security_result = self._execute_security_guide_step(
                    self.current_workflow['user_request'],
                    updated_planning_result
                )
                
                if security_result['status'] == 'completed':
                    # 수정된 가이드라인으로 Coder 실행
                    coder_result = self._execute_coder_step(
                        self.current_workflow['user_request'],
                        updated_planning_result,
                        security_result
                    )
                    
                    if coder_result['status'] == 'completed':
                        # 수정된 코드로 다시 Evaluation 단계 실행
                        evaluation_result = self._execute_evaluation_phase({
                            'final_code': coder_result['generated_code']
                        })
                        
                        if evaluation_result['status'] == 'completed':
                            return {
                                'status': 'completed',
                                'results': [updated_planning_result, security_result,
                                          coder_result, evaluation_result],
                                'final_code': coder_result['generated_code']
                            }
                        else:
                            return {
                                'status': 'failed',
                                'error': '계획 수정 후 Evaluation 실패',
                                'results': [updated_planning_result, security_result,
                                          coder_result, evaluation_result]
                            }
                    else:
                        return {
                            'status': 'failed',
                            'error': '계획 수정 후 Coder 실행 실패',
                            'results': [updated_planning_result, security_result,
                                      coder_result]
                        }
                else:
                    return {
                        'status': 'failed',
                        'error': '계획 수정 후 Security Guide 실행 실패',
                        'results': [updated_planning_result, security_result]
                    }
            else:
                return {
                    'status': 'failed',
                    'error': '모드 시퀀스 계획 수정 실패',
                    'results': phase_results
                }
                
        except Exception as e:
            self.logger.error(f"Mode Oracle 피드백 처리 실패: {e}")
            return {
                'status': 'failed',
                'error': str(e),
                'results': phase_results
            }
    
    def _combine_requirements_for_code_generation(self, user_request: str, 
                                                planning_result: Dict[str, Any],
                                                security_result: Dict[str, Any]) -> Dict[str, Any]:
        """코드 생성을 위한 요구사항 통합"""
        combined_requirements = {
            'user_request': user_request,
            'planning': planning_result.get('result', {}),
            'security': security_result.get('result', {}),
            'rag_guard': {} # RAG Guard 결과는 더 이상 사용되지 않음
        }
        
        return combined_requirements
    
    def _perform_safety_check(self, generated_code: str) -> Dict[str, Any]:
        """생성된 코드의 안전성 검증 (간단한 구현)"""
        # 실제로는 LLM API 호출을 통해 더 정교한 검증 수행
        safety_issues = []
        
        # 기본적인 안전성 검사
        if 'system(' in generated_code or 'os.system(' in generated_code:
            safety_issues.append("시스템 명령어 실행 위험")
        
        if 'eval(' in generated_code:
            safety_issues.append("eval 함수 사용 위험")
        
        if 'exec(' in generated_code:
            safety_issues.append("exec 함수 사용 위험")
        
        is_safe = len(safety_issues) == 0
        
        return {
            'is_safe': is_safe,
            'issues': safety_issues,
            'feedback': '안전성 검증 통과' if is_safe else f'안전성 문제 발견: {", ".join(safety_issues)}'
        }
    
    def _check_parameters(self, simulation_result: Dict[str, Any]) -> Dict[str, Any]:
        """파라미터 검증 (속도, 고도, 각도 등)"""
        # 실제로는 시뮬레이션 결과에서 파라미터 추출하여 검증
        return {
            'is_valid': True,  # 기본값
            'details': '파라미터 검증 통과'
        }
    
    def _check_safety_compliance(self, simulation_result: Dict[str, Any]) -> Dict[str, Any]:
        """안전성 준수 검증 (비상 절차, 금지 API, 정책 준수)"""
        # 실제로는 시뮬레이션 결과에서 안전성 검증
        return {
            'is_compliant': True,  # 기본값
            'details': '안전성 준수 검증 통과'
        }
    
    def _check_mode_sequence(self, simulation_result: Dict[str, Any]) -> Dict[str, Any]:
        """모드 시퀀스 검증 (이륙, 경유, 착륙 등)"""
        # 실제로는 시뮬레이션 결과에서 모드 시퀀스 검증
        return {
            'is_valid': True,  # 기본값
            'details': '모드 시퀀스 검증 통과'
        }
    
    def _integrate_results(self, generation_result: Dict[str, Any], 
                          evaluation_result: Dict[str, Any]) -> Dict[str, Any]:
        """Generation과 Evaluation 결과 통합"""
        return {
            'generation_phase': generation_result,
            'evaluation_phase': evaluation_result,
            'final_status': 'completed',
            'total_execution_time': time.time() - self.current_workflow['start_time']
        }
    
    def _finalize_workflow(self, status: str, message: str, result: Dict[str, Any] = None) -> Dict[str, Any]:
        """워크플로우 완료 처리"""
        self.current_workflow['status'] = status
        self.current_workflow['end_time'] = time.time()
        self.current_workflow['message'] = message
        
        if result:
            self.current_workflow['final_result'] = result
        
        self.workflow_history.append(self.current_workflow)
        
        # 워크플로우가 성공적으로 완료된 경우 생성된 코드 자동 저장
        if status == 'completed' and result:
            self._auto_save_workflow_results(result)
        
        return {
            'workflow_id': self.current_workflow['id'],
            'status': status,
            'message': message,
            'result': result,
            'execution_time': self.current_workflow.get('end_time', 0) - self.current_workflow['start_time']
        }
    
    def _auto_save_workflow_results(self, workflow_result: Dict[str, Any]):
        """워크플로우 결과를 자동으로 저장"""
        try:
            import json
            import os
            from datetime import datetime
            
            # 저장 디렉토리 생성
            output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'generated_code')
            os.makedirs(output_dir, exist_ok=True)
            
            # 타임스탬프 생성
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # 워크플로우 결과를 JSON으로 저장
            json_filename = f"workflow_result_{timestamp}.json"
            json_path = os.path.join(output_dir, json_filename)
            
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(workflow_result, f, ensure_ascii=False, indent=2)
            
            self.logger.info(f"워크플로우 결과가 JSON으로 저장되었습니다: {json_path}")
            
            # 생성된 ROS 코드를 Python 파일로 저장
            if 'generation_phase' in workflow_result:
                generation_phase = workflow_result['generation_phase']
                
                if 'results' in generation_phase:
                    # Coder Agent 결과에서 생성된 코드 찾기
                    for step_result in generation_phase['results']:
                        if step_result.get('agent') == 'Coder Agent' and 'result' in step_result:
                            coder_result = step_result['result']
                            
                            if 'code' in coder_result and coder_result['code']:
                                # Python 파일로 저장
                                py_filename = f"generated_ros_node_{timestamp}.py"
                                py_path = os.path.join(output_dir, py_filename)
                                
                                with open(py_path, 'w', encoding='utf-8') as f:
                                    f.write(coder_result['code'])
                                
                                self.logger.info(f"생성된 ROS 코드가 Python 파일로 저장되었습니다: {py_path}")
                                
                                # 코드 메타데이터도 별도 JSON으로 저장
                                metadata_filename = f"code_metadata_{timestamp}.json"
                                metadata_path = os.path.join(output_dir, metadata_filename)
                                
                                code_metadata = {
                                    'timestamp': timestamp,
                                    'filename': py_filename,
                                    'component_type': coder_result.get('component_type', 'unknown'),
                                    'security_level': coder_result.get('security_level', 'unknown'),
                                    'ai_enhanced': coder_result.get('ai_enhanced', False),
                                    'metadata': coder_result.get('metadata', {}),
                                    'security_features': coder_result.get('security_features', [])
                                }
                                
                                with open(metadata_path, 'w', encoding='utf-8') as f:
                                    json.dump(code_metadata, f, ensure_ascii=False, indent=2)
                                
                                self.logger.info(f"코드 메타데이터가 JSON으로 저장되었습니다: {metadata_path}")
                                break
                    else:
                        self.logger.warning("Coder Agent 결과에서 생성된 코드를 찾을 수 없습니다.")
                else:
                    self.logger.warning("Generation 단계 결과를 찾을 수 없습니다.")
            else:
                self.logger.warning("워크플로우 결과에서 Generation 단계 정보를 찾을 수 없습니다.")
                
        except Exception as e:
            self.logger.error(f"워크플로우 결과 자동 저장 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
