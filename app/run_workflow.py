#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sequential Workflow Execution Test
"""

import sys
import os
import logging

# Add parent directory path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from app.sequential_workflow import SequentialWorkflow

def setup_logging():
    """Setup logging"""
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
        
        # Create output directory
        output_dir = os.path.join(os.path.dirname(__file__), '..', 'data', 'generated_code')
        os.makedirs(output_dir, exist_ok=True)
        
        # Create timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save workflow result as JSON
        json_filename = f"workflow_result_{timestamp}.json"
        json_path = os.path.join(output_dir, json_filename)
        
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(workflow_result, f, ensure_ascii=False, indent=2)
        
        logger.info(f"Workflow result saved as JSON: {json_path}")
        
        # Save generated ROS code as Python file
        if 'result' in workflow_result and 'generation_phase' in workflow_result['result']:
            generation_phase = workflow_result['result']['generation_phase']
            
            if 'result' in generation_phase and 'code' in generation_phase['result']:
                generated_code = generation_phase['result']['code']
                
                if generated_code:
                    # Save as Python file
                    py_filename = f"generated_ros_node_{timestamp}.py"
                    py_path = os.path.join(output_dir, py_filename)
                    
                    with open(py_path, 'w', encoding='utf-8') as f:
                        f.write(generated_code)
                    
                    logger.info(f"Generated ROS code saved as Python file: {py_path}")
                    
                    # Save code metadata as separate JSON
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
                    
                    logger.info(f"Code metadata saved as JSON: {metadata_path}")
                else:
                    logger.warning("Generated code is empty.")
            else:
                logger.warning("Code generation result not found.")
        else:
            logger.warning("Code generation information not found in workflow result.")
            
    except Exception as e:
        logger.error(f"Error occurred while saving code: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Main execution function"""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    try:
        logger.info("Sequential workflow test started")
        
        # Initialize workflow
        workflow = SequentialWorkflow()
        
        # Initialize agents
        if not workflow.initialize_agents():
            logger.error("Agent initialization failed")
            return
        
        logger.info("All agents initialized successfully")
        
        # Test request
        test_request = """
        Create a ROS2 node that implements a simple publisher/subscriber pattern.
        The node should be able to send and receive messages through the 'hello_world' topic.
        Please generate secure code considering security.
        """
        
        logger.info(f"Test request: {test_request.strip()}")
        
        # Execute workflow
        result = workflow.execute_workflow(test_request)
        
        # Output results
        logger.info("=== Workflow Execution Results ===")
        logger.info(f"Status: {result['status']}")
        logger.info(f"Message: {result['message']}")
        logger.info(f"Execution time: {result['execution_time']:.2f} seconds")
        
        if result['status'] == 'completed':
            logger.info("Workflow completed successfully!")
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
