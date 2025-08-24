#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sequential 워크플로우 실행 테스트
"""

import sys
import os
import logging

# 상위 디렉토리 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from app.sequential_workflow import SequentialWorkflow

def setup_logging():
    """로깅 설정"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('workflow.log', encoding='utf-8')
        ]
    )

def main():
    """메인 실행 함수"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    try:
        logger.info("Sequential 워크플로우 테스트 시작")
        
        # 워크플로우 초기화
        workflow = SequentialWorkflow()
        
        # Agent들 초기화
        if not workflow.initialize_agents():
            logger.error("Agent 초기화 실패")
            return
        
        logger.info("모든 Agent 초기화 완료")
        
        # 테스트 요청
        test_request = """
        ROS2 노드를 생성하여 간단한 퍼블리셔/서브스크라이버 패턴을 구현하세요.
        노드는 'hello_world' 토픽을 통해 메시지를 주고받을 수 있어야 합니다.
        보안을 고려하여 안전한 코드를 생성해주세요.
        """
        
        logger.info(f"테스트 요청: {test_request.strip()}")
        
        # 워크플로우 실행
        result = workflow.execute_workflow(test_request)
        
        # 결과 출력
        logger.info("=== 워크플로우 실행 결과 ===")
        logger.info(f"상태: {result['status']}")
        logger.info(f"메시지: {result['message']}")
        logger.info(f"실행 시간: {result['execution_time']:.2f}초")
        
        if result['status'] == 'completed':
            logger.info("워크플로우 성공적으로 완료!")
            if 'result' in result:
                logger.info("최종 결과:")
                logger.info(f"  - Generation 단계: {result['result']['generation_phase']['status']}")
                logger.info(f"  - Evaluation 단계: {result['result']['evaluation_phase']['status']}")
        else:
            logger.error(f"워크플로우 실패: {result.get('error', '알 수 없는 오류')}")
        
    except Exception as e:
        logger.error(f"워크플로우 실행 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
