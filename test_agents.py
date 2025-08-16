#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Agent 시스템 테스트 스크립트
생성된 Planner Agent와 Security Guide Agent를 테스트
"""

import logging
import sys
import os

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def test_planner_agent():
    """Planner Agent 테스트"""
    print("=== Planner Agent 테스트 시작 ===")
    
    try:
        from agents.planner_agent import PlannerAgent
        
        # Planner Agent 생성
        planner = PlannerAgent()
        print(f"Planner Agent 생성 완료: {planner}")
        
        # 상태 확인
        status = planner.get_status()
        print(f"Agent 상태: {status}")
        
        # 간단한 요청 분석 테스트
        test_request = "ROS 2 노드를 만들어서 카메라 토픽을 구독하고, 인증 기능을 포함한 안전한 통신을 구현해줘"
        
        print(f"\n테스트 요청: {test_request}")
        
        # 요청 분석
        result = planner.plan_ros_code_generation(test_request)
        
        print("\n=== 분석 결과 ===")
        print(f"ROS 컴포넌트: {result['analysis']['ros_components']}")
        print(f"보안 요구사항: {result['analysis']['security_requirements']}")
        print(f"복잡도 레벨: {result['analysis']['complexity_level']}")
        print(f"추정 작업량: {result['analysis']['estimated_effort']}")
        
        print("\n=== 생성 계획 ===")
        print(f"필요한 Agent: {result['plan']['required_agents']}")
        print(f"예상 소요 시간: {result['plan']['estimated_time']}")
        
        for phase in result['plan']['phases']:
            print(f"  {phase['phase']}. {phase['name']} ({phase['duration']})")
        
        print("\n=== 보안 검사 항목 ===")
        for check in result['plan']['security_checks']:
            print(f"  - {check}")
        
        print("\n=== 테스트 접근법 ===")
        for approach in result['plan']['testing_approach']:
            print(f"  - {approach}")
        
        print("\n=== Planner Agent 테스트 완료 ===")
        return True
        
    except Exception as e:
        print(f"Planner Agent 테스트 실패: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_security_guide_agent():
    """Security Guide Agent 테스트"""
    print("\n=== Security Guide Agent 테스트 시작 ===")
    
    try:
        from agents.security_guide_agent import SecurityGuideAgent
        
        # Security Guide Agent 생성
        security_guide = SecurityGuideAgent()
        print(f"Security Guide Agent 생성 완료: {security_guide}")
        
        # 상태 확인
        status = security_guide.get_status()
        print(f"Agent 상태: {status}")
        
        # 보안 가이드라인 조회 테스트
        print("\n=== 보안 가이드라인 조회 테스트 ===")
        guidelines = security_guide.get_security_guidelines('authentication', 'rclpy/rclcpp')
        
        if 'error' not in guidelines:
            print("인증 카테고리 가이드라인 조회 성공")
            if 'categories' in guidelines and 'authentication' in guidelines['categories']:
                auth_guidelines = guidelines['categories']['authentication']
                print(f"  - 위험도: {auth_guidelines.get('risk_level', 'Unknown')}")
                print(f"  - CWE 수: {auth_guidelines.get('cwe_count', 0)}개")
        else:
            print(f"가이드라인 조회 실패: {guidelines['error']}")
        
        # 코드 보안 분석 테스트
        print("\n=== 코드 보안 분석 테스트 ===")
        test_code = """
        password = "admin123"
        api_key = "sk-1234567890abcdef"
        query = "SELECT * FROM users WHERE id = " + user_input
        os.system("rm -rf " + user_file)
        """
        
        print("테스트 코드:")
        print(test_code)
        
        risk_analysis = security_guide.analyze_code_security(test_code, 'rclpy/rclcpp')
        
        if 'error' not in risk_analysis:
            print(f"\n보안 위험도: {risk_analysis['risk_level']}")
            print(f"위험 점수: {risk_analysis['risk_score']}")
            
            print("\n발견된 위험:")
            for risk in risk_analysis['risks']:
                print(f"  - {risk['title']}: {risk['description']}")
                print(f"    CWE: {risk['cwe_id']}, 점수: {risk['severity_score']}")
                print(f"    완화 방안: {risk['mitigation']}")
            
            print("\n보안 권장사항:")
            for rec in risk_analysis['recommendations']:
                print(f"  - {rec}")
        else:
            print(f"보안 분석 실패: {risk_analysis['error']}")
        
        print("\n=== Security Guide Agent 테스트 완료 ===")
        return True
        
    except Exception as e:
        print(f"Security Guide Agent 테스트 실패: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_rag_guard_agent():
    """RAG Guard Agent 테스트"""
    print("\n=== RAG Guard Agent 테스트 시작 ===")
    
    try:
        from agents.rag_guard_agent import RAGGuardAgent
        
        # RAG Guard Agent 생성
        rag_guard = RAGGuardAgent()
        print(f"RAG Guard Agent 생성 완료: {rag_guard}")
        
        # 상태 확인
        status = rag_guard.get_status()
        print(f"Agent 상태: {status}")
        
        # 알고리즘 보안 검증 테스트
        print("\n=== 알고리즘 보안 검증 테스트 ===")
        test_algorithm = """
        1. 사용자 입력 받기
        2. 입력 검증 없이 직접 사용
        3. 관리자 권한으로 실행
        4. 민감한 데이터를 로그에 출력
        5. 암호화 없이 데이터 전송
        """
        
        print("테스트 알고리즘:")
        print(test_algorithm)
        
        verification_result = rag_guard._verify_algorithm_security({
            'algorithm': test_algorithm,
            'component': 'rclpy/rclcpp'
        })
        
        if 'error' not in verification_result:
            print(f"\n검증 결과: {verification_result['overall_status']}")
            print(f"보안 점수: {verification_result['security_score']}")
            print(f"검증 통과: {verification_result['verification_passed']}")
            
            print("\n발견된 이슈:")
            for issue in verification_result['issues_found']:
                print(f"  - {issue['title']}: {issue['description']}")
                print(f"    위험도 점수: {issue['severity_score']}")
                print(f"    완화 방안: {issue['mitigation']}")
            
            print("\n보안 권장사항:")
            for rec in verification_result['recommendations']:
                print(f"  - {rec}")
        else:
            print(f"검증 실패: {verification_result['error']}")
        
        # 코드 보안 검증 테스트
        print("\n=== 코드 보안 검증 테스트 ===")
        test_code = """
        password = "admin123"
        query = "SELECT * FROM users WHERE id = " + user_input
        os.system("rm -rf " + user_file)
        """
        
        print("테스트 코드:")
        print(test_code)
        
        code_verification = rag_guard._verify_code_security({
            'code_snippet': test_code,
            'component': 'rclpy/rclcpp'
        })
        
        if 'error' not in code_verification:
            print(f"\n코드 검증 결과: {code_verification['overall_status']}")
            print(f"보안 점수: {code_verification['security_score']}")
            print(f"검증 통과: {code_verification['verification_passed']}")
            
            print("\n발견된 취약점:")
            for vuln in code_verification['vulnerabilities_found']:
                print(f"  - {vuln['title']}: {vuln['description']}")
                print(f"    위험도 점수: {vuln['severity_score']}")
                print(f"    완화 방안: {vuln['mitigation']}")
            
            print("\n보안 권장사항:")
            for rec in code_verification['recommendations']:
                print(f"  - {rec}")
        else:
            print(f"코드 검증 실패: {code_verification['error']}")
        
        print("\n=== RAG Guard Agent 테스트 완료 ===")
        return True
        
    except Exception as e:
        print(f"RAG Guard Agent 테스트 실패: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_agent_communication():
    """Agent 간 통신 테스트"""
    print("\n=== Agent 간 통신 테스트 시작 ===")
    
    try:
        from agents.planner_agent import PlannerAgent
        from agents.security_guide_agent import SecurityGuideAgent
        from agents.rag_guard_agent import RAGGuardAgent
        from agents.base_agent import AgentMessage
        
        # Agent 생성
        planner = PlannerAgent()
        security_guide = SecurityGuideAgent()
        rag_guard = RAGGuardAgent()
        
        print(f"Planner Agent: {planner.agent_id}")
        print(f"Security Guide Agent: {security_guide.agent_id}")
        print(f"RAG Guard Agent: {rag_guard.agent_id}")
        
        # Planner에서 RAG Guard로 메시지 전송
        print("\n--- Planner → RAG Guard 통신 테스트 ---")
        
        message = planner.send_message(
            rag_guard.agent_id,
            'request',
            {
                'verify_algorithm': True,
                'algorithm': '사용자 입력을 받아서 직접 처리하는 알고리즘',
                'component': 'rclpy/rclcpp'
            }
        )
        
        print(f"전송된 메시지: {message.message_type}")
        print(f"내용: {message.content}")
        
        # RAG Guard에서 메시지 수신 및 처리
        response = rag_guard.receive_message(message)
        
        print(f"\n응답 메시지: {response.message_type}")
        print(f"응답 내용: {response.content}")
        
        # RAG Guard에서 Security Guide로 메시지 전송
        print("\n--- RAG Guard → Security Guide 통신 테스트 ---")
        
        message = rag_guard.send_message(
            security_guide.agent_id,
            'request',
            {
                'get_security_guidelines': True,
                'category': 'input_validation',
                'component': 'rclpy/rclcpp'
            }
        )
        
        print(f"전송된 메시지: {message.message_type}")
        print(f"내용: {message.content}")
        
        # Security Guide에서 메시지 수신 및 처리
        response = security_guide.receive_message(message)
        
        print(f"\n응답 메시지: {response.message_type}")
        print(f"응답 내용: {response.content}")
        
        print("\n=== Agent 간 통신 테스트 완료 ===")
        return True
        
    except Exception as e:
        print(f"Agent 간 통신 테스트 실패: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """메인 테스트 함수"""
    print("ROS 보안 코드 생성 시스템 - Agent 테스트 시작")
    
    # 테스트 결과
    results = []
    
    # 1. Planner Agent 테스트
    results.append(("Planner Agent", test_planner_agent()))
    
    # 2. Security Guide Agent 테스트
    results.append(("Security Guide Agent", test_security_guide_agent()))
    
    # 3. RAG Guard Agent 테스트
    results.append(("RAG Guard Agent", test_rag_guard_agent()))
    
    # 4. Agent 간 통신 테스트
    results.append(("Agent 간 통신", test_agent_communication()))
    
    # 테스트 결과 요약
    print("\n" + "="*60)
    print("테스트 결과 요약")
    print("="*60)
    
    success_count = 0
    for test_name, result in results:
        status = "성공" if result else "실패"
        print(f"{test_name}: {status}")
        if result:
            success_count += 1
    
    print(f"\n전체 테스트: {len(results)}개")
    print(f"성공: {success_count}개")
    print(f"실패: {len(results) - success_count}개")
    
    if success_count == len(results):
        print("\n모든 테스트가 성공했습니다!")
    else:
        print(f"\n{len(results) - success_count}개 테스트가 실패했습니다.")
    
    print("\n" + "="*60)

if __name__ == "__main__":
    main()
