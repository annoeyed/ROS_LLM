#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Coder Agent
AI-based Agent for generating secure ROS code
"""

import sys
import os
import re
from typing import Dict, Any, List, Optional
from .base_agent import BaseAgent, AgentMessage, AgentTask

# Add parent directory path (for accessing rag_utils module)
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class CoderAgent(BaseAgent):
    """AI-based ROS code generation Agent"""
    
    def __init__(self, agent_id: str = "coder_001"):
        super().__init__(agent_id, "Coder Agent")
        
        # Initialize AI client
        self.ai_client = None
        
        # Code generation templates
        self.code_templates = {
            'basic_node': self._get_basic_node_template(),
            'publisher': self._get_publisher_template(),
            'subscriber': self._get_subscriber_template(),
            'service': self._get_service_template(),
            'action': self._get_action_template(),
            'parameter': self._get_parameter_template()
        }
        
        # Security code patterns
        self.security_patterns = {
            'input_validation': self._get_input_validation_pattern(),
            'error_handling': self._get_error_handling_pattern(),
            'secure_logging': self._get_secure_logging_pattern(),
            'authentication': self._get_authentication_pattern(),
            'encryption': self._get_encryption_pattern()
        }
    
    def _initialize(self):
        """Initialize Coder Agent"""
        super()._initialize()
        
        try:
            # Load AI client
            self._load_ai_client()
            self.logger.info("AI client loaded successfully")
            self.logger.info("AI-based ROS code generation system initialized")
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
            
            from rag_utils.ai_client import AIClientFactory
            
            # Check AI client type from environment variables
            ai_client_type = os.getenv('AI_CLIENT_TYPE', 'mock')
            
            # Create AI client
            self.ai_client = AIClientFactory.create_client(ai_client_type)
            
            # Verify AI client loaded properly
            if self.ai_client and hasattr(self.ai_client, 'analyze_content'):
                self.logger.info(f"AI client loaded successfully: {ai_client_type}")
                # Test AI functionality
                test_response = self.ai_client.analyze_content("test", "test")
                if test_response:
                    self.logger.info("AI functionality test successful")
                else:
                    self.logger.warning("AI functionality test failed, falling back to Mock client")
                    self._fallback_to_mock_client()
            else:
                self.logger.warning("Failed to load AI client, falling back to Mock client")
                self._fallback_to_mock_client()
            
        except Exception as e:
            self.logger.error(f"Error loading AI client: {e}")
            self._fallback_to_mock_client()
    
    def _fallback_to_mock_client(self):
        """Fallback to Mock AI client"""
        try:
            from rag_utils.ai_client import MockAIClient
            self.ai_client = MockAIClient()
            self.logger.info("Fallback to Mock AI client completed")
        except Exception as mock_e:
            self.logger.error(f"Failed to load Mock AI client: {mock_e}")
            self.ai_client = None
    
    def process_message(self, message: AgentMessage) -> AgentMessage:
        """Process messages - handle code generation requests"""
        if message.message_type == 'request':
            if 'generate_code' in message.content:
                return self._handle_code_generation_request(message)
            elif 'modify_code' in message.content:
                return self._handle_code_modification_request(message)
            elif 'review_code' in message.content:
                return self._handle_code_review_request(message)
            elif 'optimize_code' in message.content:
                return self._handle_code_optimization_request(message)
            else:
                return self._handle_general_request(message)
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'Unsupported message type: {message.message_type}'}
            )
    
    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """Execute tasks - AI-based code generation"""
        self.status = 'busy'
        self.current_task = task
        
        try:
            if task.task_type == 'generate_ros_code':
                result = self._generate_ros_code(task.parameters)
            elif task.task_type == 'modify_code':
                result = self._modify_code(task.parameters)
            elif task.task_type == 'review_code':
                result = self._review_code(task.parameters)
            elif task.task_type == 'optimize_code':
                result = self._optimize_code(task.parameters)
            else:
                result = {'error': f'Unsupported task type: {task.task_type}'}
            
            self.update_task_status(task.task_id, 'completed', result)
            self.status = 'idle'
            return result
            
        except Exception as e:
            error_msg = f'Task execution failed: {str(e)}'
            self.update_task_status(task.task_id, 'failed', error=error_msg)
            self.status = 'error'
            return {'error': error_msg}
    
    def _handle_code_generation_request(self, message: AgentMessage) -> AgentMessage:
        """Handle code generation request"""
        requirements = message.content.get('requirements', '')
        component_type = message.content.get('component_type', 'basic_node')
        security_level = message.content.get('security_level', 'medium')
        
        generated_code = self._generate_ros_code({
            'requirements': requirements,
            'component_type': component_type,
            'security_level': security_level
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'generated_code': generated_code,
                'component_type': component_type,
                'security_level': security_level,
                'status': 'success'
            }
        )
    
    def _handle_code_modification_request(self, message: AgentMessage) -> AgentMessage:
        """Handle code modification request"""
        existing_code = message.content.get('existing_code', '')
        modification_request = message.content.get('modification_request', '')
        
        modified_code = self._modify_code({
            'existing_code': existing_code,
            'modification_request': modification_request
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'modified_code': modified_code,
                'modification_request': modification_request,
                'status': 'success'
            }
        )
    
    def _handle_code_review_request(self, message: AgentMessage) -> AgentMessage:
        """코드 리뷰 요청 처리"""
        code_snippet = message.content.get('code_snippet', '')
        review_focus = message.content.get('review_focus', 'security')
        
        review_result = self._review_code({
            'code_snippet': code_snippet,
            'review_focus': review_focus
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'review_result': review_result,
                'review_focus': review_focus,
                'status': 'success'
            }
        )
    
    def _handle_code_optimization_request(self, message: AgentMessage) -> AgentMessage:
        """Handle code optimization request"""
        code_snippet = message.content.get('code_snippet', '')
        optimization_goal = message.content.get('optimization_goal', 'performance')
        
        optimized_code = self._optimize_code({
            'code_snippet': code_snippet,
            'optimization_goal': optimization_goal
        })
        
        return self.send_message(
            message.sender,
            'response',
            {
                'optimized_code': optimized_code,
                'optimization_goal': optimization_goal,
                'status': 'success'
            }
        )
    
    def _handle_general_request(self, message: AgentMessage) -> AgentMessage:
        """Handle general requests"""
        return self.send_message(
            message.sender,
            'response',
            {
                'message': 'Coder Agent has processed the request.',
                'content': message.content,
                'status': 'success'
            }
        )
    
    def _generate_ros_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI-based ROS code generation"""
        requirements = parameters.get('requirements', '')
        component_type = parameters.get('component_type', 'basic_node')
        security_level = parameters.get('security_level', 'medium')
        
        if not requirements:
            return {'error': 'Requirements not provided.'}
        
        try:
            if self.ai_client:
                # AI-based code generation
                ai_generated_code = self._ai_generate_code(requirements, component_type, security_level)
                if ai_generated_code and 'error' not in ai_generated_code:
                    return {
                        'code': ai_generated_code['code'],
                        'metadata': ai_generated_code['metadata'],
                        'security_features': ai_generated_code['security_features'],
                        'ai_enhanced': True,
                        'component_type': component_type,
                        'security_level': security_level
                    }
            
            # Fallback to template-based code generation if AI fails
            template_code = self._generate_from_template(requirements, component_type, security_level)
            return {
                'code': template_code['code'],
                'metadata': template_code['metadata'],
                'security_features': template_code['security_features'],
                'ai_enhanced': False,
                'component_type': component_type,
                'security_level': security_level
            }
            
        except Exception as e:
            self.logger.error(f"Code generation failed: {e}")
            return {'error': f'Code generation failed: {str(e)}'}
    
    def _ai_generate_code(self, requirements: str, component_type: str, security_level: str) -> Dict[str, Any]:
        """AI-based code generation"""
        try:
            # AI prompt composition (more detailed and specific)
            ai_prompt = f"""
            Generate high-quality ROS 2 Python code according to the following requirements:
            
            Requirements: {requirements}
            Component type: {component_type}
            Security level: {security_level}
            
            Generate complete and safe Python code including:
            
            1. Required import statements (rclpy, std_msgs, sensor_msgs, etc.)
            2. Class definition and Node inheritance
            3. Initialization method (including security settings)
            4. Security features:
               - Input validation and sanitization
               - Error handling and recovery
               - Secure logging (sensitive information masking)
               - Authentication and authorization
               - Data encryption (when needed)
            5. Main function and exception handling
            6. Detailed comments and documentation
            7. Type hints usage
            
            Detailed requirements by security level:
            - low: Basic error handling, logging
            - medium: Input validation + error handling + secure logging + basic authentication
            - high: All security features + encryption + multi-factor authentication + audit logs
            
            ROS 2 security best practices:
            - DDS security configuration
            - Topic encryption
            - Node namespace isolation
            - QoS security settings
            
            Respond in the following JSON format:
            {{
                "code": "Complete Python code (executable state)",
                "metadata": {{
                    "description": "Detailed code description",
                    "dependencies": ["List of required packages"],
                    "usage": "Usage and execution instructions",
                    "security_features": ["List of implemented security features"],
                    "testing_notes": "Testing considerations",
                    "deployment_notes": "Deployment precautions"
                }},
                "security_analysis": {{
                    "vulnerabilities": ["Potential vulnerabilities"],
                    "mitigations": ["Mitigation strategies"],
                    "compliance": ["Security standards compliance"]
                }},
                "performance_notes": "Performance optimization points"
            }}
            
            The code must be executable in a real ROS 2 environment and follow security best practices.
            """
            
            # Execute AI code generation
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_generation")
            
            if isinstance(ai_response, dict) and 'code' in ai_response:
                # Validate and improve AI response
                validated_code = self._validate_and_improve_ai_code(ai_response['code'], security_level)
                ai_response['code'] = validated_code
                ai_response['ai_enhanced'] = True
                ai_response['quality_score'] = self._assess_code_quality(validated_code)
                
                self.logger.info(f"AI-based code generation successful (quality score: {ai_response['quality_score']})")
                return ai_response
            else:
                # AI response parsing failed
                self.logger.warning("AI response parsing failed, falling back to template-based generation")
                return {'error': 'AI response parsing failed'}
                
        except Exception as e:
            self.logger.error(f"AI-based code generation failed: {e}")
            return {'error': f'AI code generation failed: {str(e)}'}
    
    def _validate_and_improve_ai_code(self, code: str, security_level: str) -> str:
        """Validate and improve AI-generated code"""
        try:
            # Basic validation
            if not code or len(code.strip()) < 100:
                self.logger.warning("AI-generated code is too short")
                return code
            
            # Additional validation based on security level
            if security_level == 'high':
                # High security level validation
                if 'import hashlib' not in code and 'import cryptography' not in code:
                    code = self._add_encryption_imports(code)
                if 'validate_input' not in code:
                    code = self._add_input_validation(code)
                if 'secure_log' not in code:
                    code = self._add_secure_logging(code)
            
            # Code quality improvement
            code = self._improve_code_structure(code)
            code = self._add_error_handling(code)
            code = self._add_type_hints(code)
            
            return code
            
        except Exception as e:
            self.logger.error(f"Code validation and improvement failed: {e}")
            return code
    
    def _assess_code_quality(self, code: str) -> float:
        """Assess code quality"""
        try:
            score = 0.0
            max_score = 100.0
            
            # Basic structure check
            if 'class' in code and 'Node' in code:
                score += 20
            if 'def __init__' in code:
                score += 15
            if 'def main' in code:
                score += 15
            
            # Security feature check
            if 'validate_input' in code:
                score += 10
            if 'try:' in code and 'except' in code:
                score += 10
            if 'get_logger' in code:
                score += 10
            
            # Code quality check
            if '#' in code:  # Comments
                score += 5
            if '->' in code:  # Type hints
                score += 5
            if 'import' in code:
                score += 5
            if 'rclpy.init' in code and 'rclpy.shutdown' in code:
                score += 5
            
            return min(score, max_score)
            
        except Exception as e:
            self.logger.error(f"Code quality assessment failed: {e}")
            return 50.0
    
    def _add_encryption_imports(self, code: str) -> str:
        """Add encryption-related imports"""
        try:
            if 'import hashlib' not in code:
                code = "import hashlib\nimport secrets\n" + code
            return code
        except Exception as e:
            self.logger.error(f"Failed to add encryption imports: {e}")
            return code
    
    def _add_input_validation(self, code: str) -> str:
        """Add input validation method"""
        try:
            validation_method = """
    def validate_input(self, input_data: str) -> str:
        \"\"\"Input data validation\"\"\"
        if not input_data:
            raise ValueError("Input data is empty")
        
        # Type validation
        if not isinstance(input_data, str):
            raise TypeError("Input data must be a string")
        
        # Length validation
        if len(input_data) > 1000:
            raise ValueError("Input data is too long")
        
        # Special character filtering
        import re
        if re.search(r'[<>\"']', input_data):
            raise ValueError("Input data contains disallowed special characters")
        
        return input_data.strip()
"""
            # Add method inside class
            if 'class' in code and 'def' in code:
                # Add after the last method
                lines = code.split('\n')
                for i, line in enumerate(lines):
                    if line.strip().startswith('def ') and 'def __init__' not in line:
                        # Add after the first method
                        lines.insert(i, validation_method)
                        break
                code = '\n'.join(lines)
            
            return code
        except Exception as e:
            self.logger.error(f"Failed to add input validation method: {e}")
            return code
    
    def _add_secure_logging(self, code: str) -> str:
        """Add secure logging method"""
        try:
            logging_method = """
    def secure_log(self, message: str, level: str = 'info') -> None:
        \"\"\"Secure logging (sensitive information masking)\"\"\"
        # Mask sensitive information
        import re
        masked_message = re.sub(r'password[=:]\\s*\\S+', 'password=***', message)
        masked_message = re.sub(r'api_key[=:]\\s*\\S+', 'api_key=***', masked_message)
        masked_message = re.sub(r'token[=:]\\s*\\S+', 'token=***', masked_message)
        
        if level == 'info':
            self.get_logger().info(masked_message)
        elif level == 'warn':
            self.get_logger().warn(masked_message)
        elif level == 'error':
            self.get_logger().error(masked_message)
        else:
            self.get_logger().info(masked_message)
"""
            # Add method inside class
            if 'class' in code and 'def' in code:
                lines = code.split('\n')
                for i, line in enumerate(lines):
                    if line.strip().startswith('def ') and 'def __init__' not in line:
                        lines.insert(i, logging_method)
                        break
                code = '\n'.join(lines)
            
            return code
        except Exception as e:
            self.logger.error(f"Failed to add secure logging method: {e}")
            return code
    
    def _improve_code_structure(self, code: str) -> str:
        """Improve code structure"""
        try:
            # Basic structure check and improvement
            if 'if __name__ == "__main__":' not in code:
                code += '\n\nif __name__ == "__main__":\n    main()\n'
            
            return code
        except Exception as e:
            self.logger.error(f"Code structure improvement failed: {e}")
            return code
    
    def _add_error_handling(self, code: str) -> str:
        """Add error handling"""
        try:
            # Add error handling to main function
            if 'def main(' in code and 'try:' not in code:
                lines = code.split('\n')
                for i, line in enumerate(lines):
                    if 'def main(' in line:
                        # Add try-except at the beginning of main function
                        lines.insert(i + 1, '    try:')
                        lines.insert(i + 2, '        rclpy.init(args=args)')
                        lines.insert(i + 3, '        node = GenericNode()')
                        lines.insert(i + 4, '        rclpy.spin(node)')
                        lines.insert(i + 5, '    except KeyboardInterrupt:')
                        lines.insert(i + 6, '        pass')
                        lines.insert(i + 7, '    except Exception as e:')
                        lines.insert(i + 8, '        node.get_logger().error(f"Error occurred: {e}")')
                        lines.insert(i + 9, '    finally:')
                        lines.insert(i + 10, '        if "node" in locals():')
                        lines.insert(i + 11, '            node.destroy_node()')
                        lines.insert(i + 12, '        rclpy.shutdown()')
                        break
                code = '\n'.join(lines)
            
            return code
        except Exception as e:
            self.logger.error(f"Failed to add error handling: {e}")
            return code
    
    def _add_type_hints(self, code: str) -> str:
        """Add type hints"""
        try:
            # Add basic type hints
            if 'def __init__(self):' in code:
                code = code.replace('def __init__(self):', 'def __init__(self) -> None:')
            
            if 'def timer_callback(self):' in code:
                code = code.replace('def timer_callback(self):', 'def timer_callback(self) -> None:')
            
            return code
        except Exception as e:
            self.logger.error(f"Failed to add type hints: {e}")
            return code
    
    def _generate_from_template(self, requirements: str, component_type: str, security_level: str) -> Dict[str, Any]:
        """Template-based code generation"""
        try:
            # Get base template
            base_template = self.code_templates.get(component_type, self.code_templates['basic_node'])
            
            # Add security features
            security_code = self._add_security_features(base_template, security_level)
            
            # Customize according to requirements
            customized_code = self._customize_code(security_code, requirements)
            
            return {
                'code': customized_code,
                'metadata': {
                    'description': f'{component_type} based code',
                    'dependencies': ['rclpy', 'std_msgs'],
                    'usage': 'python3 generated_node.py'
                },
                'security_features': self._get_security_features(security_level)
            }
            
        except Exception as e:
            self.logger.error(f"Template-based code generation failed: {e}")
            return {'error': f'Template code generation failed: {str(e)}'}
    
    def _add_security_features(self, base_code: str, security_level: str) -> str:
        """Add security features"""
        security_code = base_code
        
        if security_level in ['medium', 'high']:
            # Add input validation
            security_code += "\n" + self.security_patterns['input_validation']
            
            # Add error handling
            security_code += "\n" + self.security_patterns['error_handling']
            
            # Add secure logging
            security_code += "\n" + self.security_patterns['secure_logging']
        
        if security_level == 'high':
            # Add authentication
            security_code += "\n" + self.security_patterns['authentication']
            
            # Add encryption
            security_code += "\n" + self.security_patterns['encryption']
        
        return security_code
    
    def _customize_code(self, base_code: str, requirements: Any) -> str:
        """Customize code according to requirements"""
        # Simple text replacement (more sophisticated parsing needed in practice)
        customized_code = base_code
        
        # Extract user_request if requirements is a dictionary
        if isinstance(requirements, dict):
            requirements_text = requirements.get('user_request', str(requirements))
        else:
            requirements_text = str(requirements)
        
        if 'camera' in requirements_text.lower():
            customized_code = customized_code.replace('GenericNode', 'CameraNode')
            customized_code = customized_code.replace('generic_topic', 'camera_topic')
        
        if 'authentication' in requirements_text.lower():
            customized_code = customized_code.replace('# TODO: Add authentication', self.security_patterns['authentication'])
        
        return customized_code
    
    def _get_security_features(self, security_level: str) -> List[str]:
        """List of features by security level"""
        if security_level == 'low':
            return ['Basic error handling']
        elif security_level == 'medium':
            return ['Input validation', 'Error handling', 'Secure logging']
        else:  # high
            return ['Input validation', 'Error handling', 'Secure logging', 'Authentication', 'Encryption']
    
    def _modify_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI-based code modification"""
        existing_code = parameters.get('existing_code', '')
        modification_request = parameters.get('modification_request', '')
        
        if not existing_code or not modification_request:
            return {'error': 'Existing code and modification request are required.'}
        
        try:
            if self.ai_client:
                # AI-based code modification
                ai_modified_code = self._ai_modify_code(existing_code, modification_request)
                if ai_modified_code and 'error' not in ai_modified_code:
                    return {
                        'modified_code': ai_modified_code['code'],
                        'changes': ai_modified_code['changes'],
                        'ai_enhanced': True
                    }
            
            # Basic modification if AI fails
            return {
                'modified_code': existing_code + f"\n# TODO: {modification_request}",
                'changes': [modification_request],
                'ai_enhanced': False
            }
            
        except Exception as e:
            self.logger.error(f"Code modification failed: {e}")
            return {'error': f'Code modification failed: {str(e)}'}
    
    def _ai_modify_code(self, existing_code: str, modification_request: str) -> Dict[str, Any]:
        """AI-based code modification"""
        try:
            ai_prompt = f"""
            Modify the following existing code according to the modification request:
            
            Existing code:
            {existing_code}
            
            Modification request: {modification_request}
            
            Provide the complete modified code and explain the changes.
            
            Respond in JSON format:
            {{
                "code": "Complete modified code",
                "changes": ["List of changes"],
                "explanation": "Modification explanation"
            }}
            """
            
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_modification")
            
            if isinstance(ai_response, dict) and 'code' in ai_response:
                return ai_response
            else:
                return {'error': 'AI response parsing failed'}
                
        except Exception as e:
            self.logger.error(f"AI-based code modification failed: {e}")
            return {'error': f'AI code modification failed: {str(e)}'}
    
    def _review_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI-based code review"""
        code_snippet = parameters.get('code_snippet', '')
        review_focus = parameters.get('review_focus', 'security')
        
        if not code_snippet:
            return {'error': 'Code snippet is required.'}
        
        try:
            if self.ai_client:
                # AI-based code review
                ai_review = self._ai_review_code(code_snippet, review_focus)
                if ai_review and 'error' not in ai_review:
                    return {
                        'review_result': ai_review,
                        'ai_enhanced': True
                    }
            
            # Basic review if AI fails
            return {
                'review_result': {
                    'score': 7,
                    'issues': ['Only basic code review performed'],
                    'recommendations': ['AI-based detailed review recommended']
                },
                'ai_enhanced': False
            }
            
        except Exception as e:
            self.logger.error(f"Code review failed: {e}")
            return {'error': f'Code review failed: {str(e)}'}
    
    def _ai_review_code(self, code_snippet: str, review_focus: str) -> Dict[str, Any]:
        """AI-based code review"""
        try:
            ai_prompt = f"""
            Review the following ROS 2 Python code from a {review_focus} perspective:
            
            Code:
            {code_snippet}
            
            Provide JSON response in the following format:
            {{
                "score": 0-10,
                "issues": ["Issues found"],
                "recommendations": ["Improvement recommendations"],
                "security_concerns": ["Security concerns"],
                "best_practices": ["Best practices applied"]
            }}
            """
            
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_review")
            
            if isinstance(ai_response, dict) and 'score' in ai_response:
                return ai_response
            else:
                return {'error': 'AI response parsing failed'}
                
        except Exception as e:
            self.logger.error(f"AI-based code review failed: {e}")
            return {'error': f'AI code review failed: {str(e)}'}
    
    def _optimize_code(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """AI-based code optimization"""
        code_snippet = parameters.get('code_snippet', '')
        optimization_goal = parameters.get('optimization_goal', 'performance')
        
        if not code_snippet:
            return {'error': 'Code snippet is required.'}
        
        try:
            if self.ai_client:
                # AI-based code optimization
                ai_optimized = self._ai_optimize_code(code_snippet, optimization_goal)
                if ai_optimized and 'error' not in ai_optimized:
                    return {
                        'optimized_code': ai_optimized['code'],
                        'optimizations': ai_optimized['optimizations'],
                        'ai_enhanced': True
                    }
            
            # Basic optimization if AI fails
            return {
                'optimized_code': code_snippet + f"\n# TODO: {optimization_goal} optimization needed",
                'optimizations': [f'{optimization_goal} optimization'],
                'ai_enhanced': False
            }
            
        except Exception as e:
            self.logger.error(f"Code optimization failed: {e}")
            return {'error': f'Code optimization failed: {str(e)}'}
    
    def _ai_optimize_code(self, code_snippet: str, optimization_goal: str) -> Dict[str, Any]:
        """AI-based code optimization"""
        try:
            ai_prompt = f"""
            Optimize the following ROS 2 Python code from a {optimization_goal} perspective:
            
            Code:
            {code_snippet}
            
            Provide optimized code and explain the optimization techniques applied.
            
            Respond in JSON format:
            {{
                "code": "Optimized code",
                "optimizations": ["Optimization techniques applied"],
                "performance_improvements": ["Performance improvements"],
                "trade_offs": ["Trade-off considerations"]
            }}
            """
            
            ai_response = self.ai_client.analyze_content(ai_prompt, "code_optimization")
            
            if isinstance(ai_response, dict) and 'code' in ai_response:
                return ai_response
            else:
                return {'error': 'AI response parsing failed'}
                
        except Exception as e:
            self.logger.error(f"AI-based code optimization failed: {e}")
            return {'error': f'AI code optimization failed: {str(e)}'}
    
    # Code template methods
    def _get_basic_node_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GenericNode(Node):
    def __init__(self):
        super().__init__('generic_node')
        self.publisher = self.create_publisher(String, 'generic_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Generic node has been started')
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = GenericNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_publisher_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Publisher node has been started')
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Published message'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_subscriber_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.get_logger().info('Subscriber node has been started')
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_service_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(Trigger, 'trigger_service', self.trigger_callback)
        self.get_logger().info('Service node has been started')
    
    def trigger_callback(self, request, response):
        self.get_logger().info('Service called')
        response.success = True
        response.message = 'Service executed successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_action_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Action node has been started')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # Action execution logic here
        goal_handle.succeed()
        result = Fibonacci.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    def _get_parameter_template(self) -> str:
        return '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'default_value')
        self.parameter_value = self.get_parameter('my_parameter').value
        self.get_logger().info(f'Parameter value: {self.parameter_value}')
        self.get_logger().info('Parameter node has been started')
    
    def get_parameter_value(self):
        return self.parameter_value

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
    
    # Security pattern methods
    def _get_input_validation_pattern(self) -> str:
        return '''
def validate_input(self, input_data):
    """Input data validation"""
    if not input_data:
        raise ValueError("Input data is empty")
    
    # Type validation
    if not isinstance(input_data, str):
        raise TypeError("Input data must be a string")
    
    # Length validation
    if len(input_data) > 1000:
        raise ValueError("Input data is too long")
    
    # Special character filtering
    import re
    if re.search(r'[<>"\']', input_data):
        raise ValueError("Input data contains disallowed special characters")
    
    return input_data.strip()'''
    
    def _get_error_handling_pattern(self) -> str:
        return '''
def safe_execute(self, operation, *args, **kwargs):
    """Safe operation execution"""
    try:
        result = operation(*args, **kwargs)
        return result
    except Exception as e:
        self.get_logger().error(f"Operation execution failed: {e}")
        # Return default value or handle error
        return None
    
def handle_critical_error(self, error):
    """Critical error handling"""
    self.get_logger().error(f"Critical error occurred: {error}")
    # Transition to system safe state
    self.emergency_shutdown()'''
    
    def _get_secure_logging_pattern(self) -> str:
        return '''
def secure_log(self, message, level='info'):
    """Secure logging"""
    # Mask sensitive information
    import re
    masked_message = re.sub(r'password[=:]\s*\S+', 'password=***', message)
    masked_message = re.sub(r'api_key[=:]\s*\S+', 'api_key=***', masked_message)
    
    if level == 'info':
        self.get_logger().info(masked_message)
    elif level == 'warn':
        self.get_logger().warn(masked_message)
    elif level == 'error':
        self.get_logger().error(masked_message)'''
    
    def _get_authentication_pattern(self) -> str:
        return '''
def authenticate_user(self, credentials):
    """User authentication"""
    # TODO: Add authentication
    if not credentials:
        return False
    
    # Implement authentication logic
    return True
    
def check_permission(self, user, resource):
    """Permission check"""
    # TODO: Add permission check
    return True'''
    
    def _get_encryption_pattern(self) -> str:
        return '''
def encrypt_data(self, data):
    """Data encryption"""
    # TODO: Add encryption
    import hashlib
    return hashlib.sha256(data.encode()).hexdigest()
    
def decrypt_data(self, encrypted_data):
    """Data decryption"""
    # TODO: Add decryption
    return encrypted_data'''
    
    def get_status(self) -> Dict[str, Any]:
        """Query Agent status"""
        return {
            'agent_id': self.agent_id,
            'agent_name': self.agent_name,
            'status': self.status,
            'message_queue_length': len(self.message_queue),
            'task_history_length': len(self.task_history),
            'current_task': self.current_task.task_id if self.current_task else None,
            'ai_client': {
                'loaded': self.ai_client is not None,
                'type': self.ai_client.__class__.__name__ if self.ai_client else None,
                'enhanced_generation': self.ai_client is not None
            },
            'code_templates': list(self.code_templates.keys()),
            'security_patterns': list(self.security_patterns.keys()