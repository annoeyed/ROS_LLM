#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Agent 워크플로우 시스템
AI 기반으로 모든 Agent들이 협력하여 ROS 보안 코드를 생성하는 통합 시스템
"""

import sys
import os
import json
import logging
import time
from typing import Dict, Any, List, Optional
from concurrent.futures import ThreadPoolExecutor, as_completed

# 상위 디렉토리 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from agents import (
    PlannerAgent, 
    SecurityGuideAgent, 
    RAGGuardAgent, 
    CoderAgent, 
    SimulationAgent
)
from agents.base_agent import AgentMessage, AgentTask

class MultiAgentWorkflow:
    """AI 기반 Multi-Agent 워크플로우 시스템"""
    
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Agent들 초기화
        self.agents = {}
        self.workflow_history = []
        self.current_workflow = None
        
        # AI 클라이언트 설정
        self.ai_client_type = os.getenv('AI_CLIENT_TYPE', 'mock')
        
        # 워크플로우 설정
        self.workflow_config = {
            'max_retries': 3,
            'timeout': 300,  # 5분
            'parallel_execution': True,
            'ai_enhanced': True
        }
    
    def initialize_agents(self):
        """모든 Agent 초기화"""
        try:
            self.logger.info("Multi-Agent 시스템 초기화 시작")
            
            # 1. Planner Agent (AI 기반 계획 수립)
            self.agents['planner'] = PlannerAgent("planner_001")
            self.logger.info("Planner Agent 초기화 완료")
            
            # 2. Security Guide Agent (CWE 기반 보안 가이드라인)
            self.agents['security_guide'] = SecurityGuideAgent("security_guide_001")
            self.logger.info("Security Guide Agent 초기화 완료")
            
            # 3. RAG Guard Agent (AI + RAG 기반 보안 검증)
            self.agents['rag_guard'] = RAGGuardAgent("rag_guard_001")
            self.logger.info("RAG Guard Agent 초기화 완료")
            
            # 4. Coder Agent (AI 기반 코드 생성)
            self.agents['coder'] = CoderAgent("coder_001")
            self.logger.info("Coder Agent 초기화 완료")
            
            # 5. Simulation Agent (AI 기반 시뮬레이션 및 테스트)
            self.agents['simulation'] = SimulationAgent("simulation_001")
            self.logger.info("Simulation Agent 초기화 완료")
            
            self.logger.info("모든 Agent 초기화 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"Agent 초기화 실패: {e}")
            return False
    
    def execute_workflow(self, user_request: str, workflow_type: str = "full_development") -> Dict[str, Any]:
        """AI 기반 Multi-Agent 워크플로우 실행"""
        workflow_id = f"workflow_{int(time.time())}"
        self.current_workflow = {
            'id': workflow_id,
            'type': workflow_type,
            'user_request': user_request,
            'start_time': time.time(),
            'status': 'running',
            'steps': []
        }
        
        try:
            self.logger.info(f"워크플로우 시작: {workflow_id}")
            
            if workflow_type == "full_development":
                result = self._execute_full_development_workflow(user_request)
            elif workflow_type == "security_audit":
                result = self._execute_security_audit_workflow(user_request)
            elif workflow_type == "code_generation":
                result = self._execute_code_generation_workflow(user_request)
            elif workflow_type == "testing_only":
                result = self._execute_testing_workflow(user_request)
            else:
                result = {'error': f'지원하지 않는 워크플로우 타입: {workflow_type}'}
            
            # 워크플로우 완료
            self.current_workflow['status'] = 'completed'
            self.current_workflow['end_time'] = time.time()
            self.current_workflow['duration'] = self.current_workflow['end_time'] - self.current_workflow['start_time']
            self.current_workflow['result'] = result
            
            # 워크플로우 히스토리에 추가
            self.workflow_history.append(self.current_workflow.copy())
            
            return {
                'workflow_id': workflow_id,
                'status': 'completed',
                'result': result,
                'duration': self.current_workflow['duration']
            }
            
        except Exception as e:
            self.logger.error(f"워크플로우 실행 실패: {e}")
            self.current_workflow['status'] = 'failed'
            self.current_workflow['error'] = str(e)
            self.current_workflow['end_time'] = time.time()
            
            return {
                'workflow_id': workflow_id,
                'status': 'failed',
                'error': str(e)
            }
    
    def _execute_full_development_workflow(self, user_request: str) -> Dict[str, Any]:
        """전체 개발 워크플로우 실행"""
        self.logger.info("전체 개발 워크플로우 시작")
        
        workflow_steps = []
        
        # 1단계: AI 기반 계획 수립 (Planner Agent)
        self.logger.info("1단계: AI 기반 계획 수립")
        planning_step = self._execute_planning_step(user_request)
        workflow_steps.append(planning_step)
        
        if planning_step['status'] == 'failed':
            return {'error': '계획 수립 실패', 'workflow_steps': workflow_steps}
        
        # 2단계: 보안 가이드라인 생성 (Security Guide Agent)
        self.logger.info("2단계: 보안 가이드라인 생성")
        security_step = self._execute_security_guidelines_step(user_request, planning_step['result'])
        workflow_steps.append(security_step)
        
        # 3단계: 보안 검증 (RAG Guard Agent)
        self.logger.info("3단계: AI + RAG 기반 보안 검증")
        security_verification_step = self._execute_security_verification_step(user_request, planning_step['result'])
        workflow_steps.append(security_verification_step)
        
        # 4단계: AI 기반 코드 생성 (Coder Agent)
        self.logger.info("4단계: AI 기반 보안 코드 생성")
        code_generation_step = self._execute_code_generation_step(user_request, planning_step['result'], security_step['result'])
        workflow_steps.append(code_generation_step)
        
        if code_generation_step['status'] == 'failed':
            return {'error': '코드 생성 실패', 'workflow_steps': workflow_steps}
        
        # 5단계: AI 기반 시뮬레이션 및 테스트 (Simulation Agent)
        self.logger.info("5단계: AI 기반 시뮬레이션 및 테스트")
        simulation_step = self._execute_simulation_step(code_generation_step['result'])
        workflow_steps.append(simulation_step)
        
        # 6단계: 최종 검증 및 통합
        self.logger.info("6단계: 최종 검증 및 통합")
        final_verification_step = self._execute_final_verification_step(workflow_steps)
        workflow_steps.append(final_verification_step)
        
        return {
            'workflow_steps': workflow_steps,
            'final_result': {
                'generated_code': code_generation_step['result'],
                'security_analysis': security_verification_step['result'],
                'test_results': simulation_step['result'],
                'overall_status': 'completed'
            }
        }
    
    def _execute_planning_step(self, user_request: str) -> Dict[str, Any]:
        """AI 기반 계획 수립 단계"""
        try:
            start_time = time.time()
            
            # Planner Agent에 계획 수립 요청
            planner_agent = self.agents['planner']
            planning_result = planner_agent.plan_ros_code_generation(user_request)
            
            execution_time = time.time() - start_time
            
            return {
                'step_name': 'AI 기반 계획 수립',
                'agent': 'Planner Agent',
                'status': 'completed' if 'error' not in planning_result else 'failed',
                'result': planning_result,
                'execution_time': execution_time,
                'ai_enhanced': planning_result.get('ai_enhanced', False)
            }
            
        except Exception as e:
            self.logger.error(f"계획 수립 단계 실패: {e}")
            return {
                'step_name': 'AI 기반 계획 수립',
                'agent': 'Planner Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_security_guidelines_step(self, user_request: str, planning_result: Dict[str, Any]) -> Dict[str, Any]:
        """보안 가이드라인 생성 단계"""
        try:
            start_time = time.time()
            
            # Security Guide Agent에 가이드라인 요청
            security_agent = self.agents['security_guide']
            
            # 계획 결과에서 보안 요구사항 추출
            security_requirements = planning_result.get('analysis', {}).get('security_requirements', [])
            
            # 보안 가이드라인 요청
            guidelines_result = security_agent.get_security_guidelines('general', 'ros_node')
            
            execution_time = time.time() - start_time
            
            return {
                'step_name': '보안 가이드라인 생성',
                'agent': 'Security Guide Agent',
                'status': 'completed' if 'error' not in guidelines_result else 'failed',
                'result': guidelines_result,
                'execution_time': execution_time,
                'ai_enhanced': False  # Security Guide는 CWE 기반
            }
            
        except Exception as e:
            self.logger.error(f"보안 가이드라인 단계 실패: {e}")
            return {
                'step_name': '보안 가이드라인 생성',
                'agent': 'Security Guide Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_security_verification_step(self, user_request: str, planning_result: Dict[str, Any]) -> Dict[str, Any]:
        """AI + RAG 기반 보안 검증 단계"""
        try:
            start_time = time.time()
            
            # RAG Guard Agent에 보안 검증 요청
            rag_guard_agent = self.agents['rag_guard']
            
            # 계획 결과에서 알고리즘 정보 추출
            algorithm_description = planning_result.get('analysis', {}).get('technical_details', {}).get('description', user_request)
            
            security_verification_result = rag_guard_agent._rag_based_security_verification(
                algorithm_description, 'ros_node'
            )
            
            execution_time = time.time() - start_time
            
            return {
                'step_name': 'AI + RAG 기반 보안 검증',
                'agent': 'RAG Guard Agent',
                'status': 'completed' if 'error' not in security_verification_result else 'failed',
                'result': security_verification_result,
                'execution_time': execution_time,
                'ai_enhanced': security_verification_result.get('ai_enhanced', False)
            }
            
        except Exception as e:
            self.logger.error(f"보안 검증 단계 실패: {e}")
            return {
                'step_name': 'AI + RAG 기반 보안 검증',
                'agent': 'RAG Guard Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_code_generation_step(self, user_request: str, planning_result: Dict[str, Any], security_guidelines: Dict[str, Any]) -> Dict[str, Any]:
        """AI 기반 코드 생성 단계"""
        try:
            start_time = time.time()
            
            # Coder Agent에 코드 생성 요청
            coder_agent = self.agents['coder']
            
            # 계획 결과와 보안 가이드라인을 결합하여 코드 생성 요청
            code_generation_request = self._combine_requirements_for_code_generation(
                user_request, planning_result, security_guidelines
            )
            
            code_generation_result = coder_agent._generate_ros_code({
                'requirements': code_generation_request,
                'component_type': 'basic_node',
                'security_level': 'high'
            })
            
            execution_time = time.time() - start_time
            
            return {
                'step_name': 'AI 기반 보안 코드 생성',
                'agent': 'Coder Agent',
                'status': 'completed' if 'error' not in code_generation_result else 'failed',
                'result': code_generation_result,
                'execution_time': execution_time,
                'ai_enhanced': code_generation_result.get('ai_enhanced', False)
            }
            
        except Exception as e:
            self.logger.error(f"코드 생성 단계 실패: {e}")
            return {
                'step_name': 'AI 기반 보안 코드 생성',
                'agent': 'Coder Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_simulation_step(self, generated_code: Dict[str, Any]) -> Dict[str, Any]:
        """AI 기반 시뮬레이션 및 테스트 단계"""
        try:
            start_time = time.time()
            
            # Simulation Agent에 시뮬레이션 요청
            simulation_agent = self.agents['simulation']
            
            # 코드 스니펫 추출 (Coder Agent 결과 구조에 맞춤)
            if isinstance(generated_code, dict):
                code_snippet = generated_code.get('code', '')
            else:
                code_snippet = str(generated_code)
            
            # 종합 테스트 실행
            test_result = simulation_agent.run_comprehensive_tests({
                'code_snippet': code_snippet,
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
                'execution_time': execution_time,
                'ai_enhanced': test_result.get('ai_enhanced', False)
            }
            
        except Exception as e:
            self.logger.error(f"시뮬레이션 단계 실패: {e}")
            return {
                'step_name': 'AI 기반 시뮬레이션 및 테스트',
                'agent': 'Simulation Agent',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _execute_final_verification_step(self, workflow_steps: List[Dict[str, Any]]) -> Dict[str, Any]:
        """최종 검증 및 통합 단계"""
        try:
            start_time = time.time()
            
            # 모든 단계 결과를 종합하여 최종 검증
            verification_result = self._comprehensive_verification(workflow_steps)
            
            execution_time = time.time() - start_time
            
            return {
                'step_name': '최종 검증 및 통합',
                'agent': 'Multi-Agent System',
                'status': 'completed',
                'result': verification_result,
                'execution_time': execution_time,
                'ai_enhanced': True
            }
            
        except Exception as e:
            self.logger.error(f"최종 검증 단계 실패: {e}")
            return {
                'step_name': '최종 검증 및 통합',
                'agent': 'Multi-Agent System',
                'status': 'failed',
                'error': str(e),
                'execution_time': 0.0
            }
    
    def _combine_requirements_for_code_generation(self, user_request: str, planning_result: Dict[str, Any], security_guidelines: Dict[str, Any]) -> str:
        """코드 생성을 위한 요구사항 통합"""
        combined_requirements = f"""
        원본 요청: {user_request}
        
        계획된 기능:
        {json.dumps(planning_result.get('plan', {}), indent=2, ensure_ascii=False)}
        
        보안 요구사항:
        {json.dumps(security_guidelines.get('guidelines', {}), indent=2, ensure_ascii=False)}
        
        다음을 포함하여 보안을 고려한 ROS 2 코드를 생성하세요:
        1. 계획된 모든 기능 구현
        2. 보안 가이드라인 준수
        3. 입력 검증 및 에러 처리
        4. 보안 로깅 및 모니터링
        5. 안전한 기본값 설정
        """
        
        return combined_requirements
    
    def _comprehensive_verification(self, workflow_steps: List[Dict[str, Any]]) -> Dict[str, Any]:
        """종합 검증 수행"""
        try:
            # 각 단계별 상태 확인
            completed_steps = [step for step in workflow_steps if step.get('status') == 'completed']
            failed_steps = [step for step in workflow_steps if step.get('status') == 'failed']
            
            # AI 향상도 계산
            ai_enhanced_steps = [step for step in completed_steps if step.get('ai_enhanced', False)]
            ai_enhancement_rate = (len(ai_enhanced_steps) / len(completed_steps)) * 100 if completed_steps else 0
            
            # 전체 성공률 계산
            success_rate = (len(completed_steps) / len(workflow_steps)) * 100 if workflow_steps else 0
            
            # 보안 점수 계산
            security_score = self._calculate_security_score(workflow_steps)
            
            verification_result = {
                'overall_status': 'success' if success_rate >= 80 else 'partial_success' if success_rate >= 60 else 'failed',
                'success_rate': success_rate,
                'completed_steps': len(completed_steps),
                'failed_steps': len(failed_steps),
                'ai_enhancement_rate': ai_enhancement_rate,
                'security_score': security_score,
                'recommendations': self._generate_recommendations(workflow_steps),
                'quality_metrics': {
                    'code_quality': 'high' if security_score >= 80 else 'medium' if security_score >= 60 else 'low',
                    'security_compliance': 'compliant' if security_score >= 80 else 'partially_compliant' if security_score >= 60 else 'non_compliant',
                    'ai_integration': 'fully_integrated' if ai_enhancement_rate >= 80 else 'partially_integrated' if ai_enhancement_rate >= 60 else 'minimal_integration'
                }
            }
            
            return verification_result
            
        except Exception as e:
            self.logger.error(f"종합 검증 실패: {e}")
            return {
                'overall_status': 'failed',
                'error': str(e),
                'success_rate': 0.0
            }
    
    def _calculate_security_score(self, workflow_steps: List[Dict[str, Any]]) -> float:
        """보안 점수 계산"""
        try:
            total_score = 0
            max_score = 0
            
            for step in workflow_steps:
                if step.get('status') == 'completed':
                    result = step.get('result', {})
                    
                    if 'security_score' in result:
                        total_score += result['security_score']
                        max_score += 100
                    elif 'overall_status' in result:
                        if result['overall_status'] == 'pass':
                            total_score += 90
                        elif result['overall_status'] == 'warning':
                            total_score += 70
                        elif result['overall_status'] == 'fail':
                            total_score += 30
                        max_score += 100
            
            return (total_score / max_score) * 100 if max_score > 0 else 0.0
            
        except Exception as e:
            self.logger.error(f"보안 점수 계산 실패: {e}")
            return 0.0
    
    def _generate_recommendations(self, workflow_steps: List[Dict[str, Any]]) -> List[str]:
        """개선 권장사항 생성"""
        recommendations = []
        
        try:
            # AI 향상도 기반 권장사항
            ai_enhanced_steps = [step for step in workflow_steps if step.get('ai_enhanced', False)]
            if len(ai_enhanced_steps) < len(workflow_steps) * 0.8:
                recommendations.append("더 많은 단계에서 AI 기능을 활용하여 품질 향상")
            
            # 보안 기반 권장사항
            security_steps = [step for step in workflow_steps if 'security' in step.get('step_name', '').lower()]
            if security_steps:
                failed_security_steps = [step for step in security_steps if step.get('status') == 'failed']
                if failed_security_steps:
                    recommendations.append("보안 검증 단계 강화 및 재검토 필요")
            
            # 성능 기반 권장사항
            slow_steps = [step for step in workflow_steps if step.get('execution_time', 0) > 10]
            if slow_steps:
                recommendations.append("실행 시간이 긴 단계 최적화 검토")
            
            # 기본 권장사항
            if not recommendations:
                recommendations.append("현재 워크플로우가 최적 상태로 실행됨")
                recommendations.append("정기적인 보안 업데이트 및 테스트 수행 권장")
            
        except Exception as e:
            self.logger.error(f"권장사항 생성 실패: {e}")
            recommendations.append("권장사항 생성 중 오류 발생")
        
        return recommendations
    
    def _execute_security_audit_workflow(self, user_request: str) -> Dict[str, Any]:
        """보안 감사 전용 워크플로우"""
        self.logger.info("보안 감사 워크플로우 시작")
        
        workflow_steps = []
        
        # 1단계: 보안 가이드라인 생성
        security_step = self._execute_security_guidelines_step(user_request, {})
        workflow_steps.append(security_step)
        
        # 2단계: 보안 검증
        security_verification_step = self._execute_security_verification_step(user_request, {})
        workflow_steps.append(security_verification_step)
        
        return {
            'workflow_steps': workflow_steps,
            'final_result': {
                'security_analysis': security_verification_step['result'],
                'guidelines': security_step['result'],
                'overall_status': 'completed'
            }
        }
    
    def _execute_code_generation_workflow(self, user_request: str) -> Dict[str, Any]:
        """코드 생성 전용 워크플로우"""
        self.logger.info("코드 생성 워크플로우 시작")
        
        workflow_steps = []
        
        # 1단계: 계획 수립
        planning_step = self._execute_planning_step(user_request)
        workflow_steps.append(planning_step)
        
        # 2단계: 코드 생성
        code_generation_step = self._execute_code_generation_step(user_request, planning_step['result'], {})
        workflow_steps.append(code_generation_step)
        
        return {
            'workflow_steps': workflow_steps,
            'final_result': {
                'generated_code': code_generation_step['result']['generated_code'],
                'overall_status': 'completed'
            }
        }
    
    def _execute_testing_workflow(self, user_request: str) -> Dict[str, Any]:
        """테스트 전용 워크플로우"""
        self.logger.info("테스트 워크플로우 시작")
        
        workflow_steps = []
        
        # 1단계: 시뮬레이션 및 테스트
        simulation_step = self._execute_simulation_step({'code': user_request})
        workflow_steps.append(simulation_step)
        
        return {
            'workflow_steps': workflow_steps,
            'final_result': {
                'test_results': simulation_step['result'],
                'overall_status': 'completed'
            }
        }
    
    def get_workflow_status(self, workflow_id: str = None) -> Dict[str, Any]:
        """워크플로우 상태 조회"""
        if workflow_id:
            # 특정 워크플로우 상태 조회
            for workflow in self.workflow_history:
                if workflow['id'] == workflow_id:
                    return workflow
            return {'error': f'워크플로우를 찾을 수 없음: {workflow_id}'}
        else:
            # 현재 워크플로우 상태 조회
            if self.current_workflow:
                return self.current_workflow
            else:
                return {'status': 'no_active_workflow'}
    
    def get_agent_statuses(self) -> Dict[str, Any]:
        """모든 Agent 상태 조회"""
        agent_statuses = {}
        
        for agent_name, agent in self.agents.items():
            try:
                agent_statuses[agent_name] = agent.get_status()
            except Exception as e:
                agent_statuses[agent_name] = {
                    'error': f'상태 조회 실패: {str(e)}',
                    'status': 'unknown'
                }
        
        return agent_statuses
    
    def get_workflow_history(self) -> List[Dict[str, Any]]:
        """워크플로우 히스토리 조회"""
        return self.workflow_history.copy()
    
    def get_system_summary(self) -> Dict[str, Any]:
        """시스템 전체 요약"""
        try:
            total_workflows = len(self.workflow_history)
            successful_workflows = len([w for w in self.workflow_history if w['status'] == 'completed'])
            failed_workflows = len([w for w in self.workflow_history if w['status'] == 'failed'])
            
            # AI 향상도 계산
            ai_enhanced_workflows = 0
            for workflow in self.workflow_history:
                if workflow['status'] == 'completed' and 'result' in workflow:
                    workflow_steps = workflow['result'].get('workflow_steps', [])
                    ai_enhanced_steps = [step for step in workflow_steps if step.get('ai_enhanced', False)]
                    if len(ai_enhanced_steps) >= len(workflow_steps) * 0.5:
                        ai_enhanced_workflows += 1
            
            ai_enhancement_rate = (ai_enhanced_workflows / total_workflows) * 100 if total_workflows > 0 else 0
            
            return {
                'total_workflows': total_workflows,
                'successful_workflows': successful_workflows,
                'failed_workflows': failed_workflows,
                'success_rate': (successful_workflows / total_workflows) * 100 if total_workflows > 0 else 0,
                'ai_enhanced_workflows': ai_enhanced_workflows,
                'ai_enhancement_rate': ai_enhancement_rate,
                'active_agents': len(self.agents),
                'system_status': 'healthy' if failed_workflows < total_workflows * 0.2 else 'degraded',
                'last_workflow': self.workflow_history[-1] if self.workflow_history else None
            }
            
        except Exception as e:
            self.logger.error(f"시스템 요약 생성 실패: {e}")
            return {
                'error': f'시스템 요약 생성 실패: {str(e)}',
                'system_status': 'error'
            }

# 사용 예시
if __name__ == "__main__":
    # 로깅 설정
    logging.basicConfig(level=logging.INFO)
    
    # Multi-Agent 워크플로우 시스템 생성
    workflow_system = MultiAgentWorkflow()
    
    # Agent 초기화
    if workflow_system.initialize_agents():
        print("Multi-Agent 시스템 초기화 성공")
        
        # 전체 개발 워크플로우 실행
        user_request = "ROS 2 기반 보안 카메라 노드를 생성해주세요. 인증 기능과 암호화가 포함되어야 합니다."
        
        result = workflow_system.execute_workflow(user_request, "full_development")
        
        print(f"워크플로우 실행 결과: {json.dumps(result, indent=2, ensure_ascii=False)}")
        
        # 시스템 요약 조회
        summary = workflow_system.get_system_summary()
        print(f"시스템 요약: {json.dumps(summary, indent=2, ensure_ascii=False)}")
        
    else:
        print("Multi-Agent 시스템 초기화 실패")
