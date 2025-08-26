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
        format='%(asctime)s - %(name)s - %(levelname)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('workflow.log', encoding='utf-8')
        ]
    )

def save_generated_code(workflow_result, logger):
    """생성된 코드를 자동으로 저장"""
    try:
        import json
        import os
        from datetime import datetime
        
        # 저장 디렉토리 생성
        output_dir = os.path.join(os.path.dirname(__file__), '..', 'data', 'generated_code')
        os.makedirs(output_dir, exist_ok=True)
        
        # 타임스탬프 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 워크플로우 결과를 JSON으로 저장
        json_filename = f"workflow_result_{timestamp}.json"
        json_path = os.path.join(output_dir, json_filename)
        
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(workflow_result, f, ensure_ascii=False, indent=2)
        
        logger.info(f"워크플로우 결과가 JSON으로 저장되었습니다: {json_path}")
        
        # 생성된 ROS 코드를 Python 파일로 저장
        if 'result' in workflow_result and 'generation_phase' in workflow_result['result']:
            generation_phase = workflow_result['result']['generation_phase']
            
            if 'result' in generation_phase and 'code' in generation_phase['result']:
                generated_code = generation_phase['result']['code']
                
                if generated_code:
                    # Python 파일로 저장
                    py_filename = f"generated_ros_node_{timestamp}.py"
                    py_path = os.path.join(output_dir, py_filename)
                    
                    with open(py_path, 'w', encoding='utf-8') as f:
                        f.write(generated_code)
                    
                    logger.info(f"생성된 ROS 코드가 Python 파일로 저장되었습니다: {py_path}")
                    
                    # 코드 메타데이터도 별도 JSON으로 저장
                    metadata_filename = f"code_metadata_{timestamp}.json"
                    metadata_path = os.path.join(output_dir, metadata_filename)
                    
                    code_metadata = {
                        'timestamp': timestamp,
                        'filename': py_filename,
                        'component_type': generation_phase['result'].get('component_type', 'unknown'),
                        'security_level': generation_phase['result'].get('security_level', 'unknown'),
                        'ai_enhanced': generation_phase['result'].get('ai_enhanced', False),
                        'metadata': generation_phase['result'].get('metadata', {}),
                        'security_features': generation_phase['result'].get('security_features', [])
                    }
                    
                    with open(metadata_path, 'w', encoding='utf-8') as f:
                        json.dump(code_metadata, f, ensure_ascii=False, indent=2)
                    
                    logger.info(f"코드 메타데이터가 JSON으로 저장되었습니다: {metadata_path}")
                else:
                    logger.warning("생성된 코드가 비어있습니다.")
            else:
                logger.warning("코드 생성 결과를 찾을 수 없습니다.")
        else:
            logger.warning("워크플로우 결과에서 코드 생성 정보를 찾을 수 없습니다.")
            
    except Exception as e:
        logger.error(f"코드 저장 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()

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
                
                # 생성된 코드 자동 저장
                save_generated_code(result, logger)
        else:
            logger.error(f"워크플로우 실패: {result.get('error', '알 수 없는 오류')}")
        
    except Exception as e:
        logger.error(f"워크플로우 실행 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
