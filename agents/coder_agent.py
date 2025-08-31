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
    
    def __init__(self, llm_client, agent_id: str = "coder_001"):
        super().__init__(agent_id, "Coder Agent")
        # Initialize AI client
        self.llm_client = llm_client
        self.ai_client = llm_client  # Use the provided LLM client directly
        
        # Code generation templates - Python
        self.python_templates = {
            'basic_node': self._get_basic_node_template(),
            'publisher': self._get_publisher_template(),
            'subscriber': self._get_subscriber_template(),
            'service': self._get_service_template(),
            'action': self._get_action_template(),
            'parameter': self._get_parameter_template()
        }
        
        # Code generation templates - C++ (simplified)
        self.cpp_templates = {
            'basic_node': self._get_cpp_basic_node_template(),  
            'publisher': self._get_cpp_publisher_template(),   
            'subscriber': self._get_cpp_subscriber_template(),  
            'service': self._get_cpp_service_template(),     
            'parameter': self._get_cpp_parameter_template()    
        }
        
        # C 언어 템플릿 제거됨
        
        # Combined templates for backward compatibility
        self.code_templates = self.python_templates
        
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
            
            from utils.ai_client import AIClientFactory
            
            # Check AI client type from environment variables
            ai_client_type = os.getenv('AI_CLIENT_TYPE')
            
            # AI client is already set from constructor parameter
            # No need to create a new one here
            
            # Verify AI client loaded properly
            if self.ai_client and hasattr(self.ai_client, 'generate_response'):
                self.logger.info(f"AI client loaded successfully: {ai_client_type}")
            else:
                self.logger.error("AI client not properly initialized")
                raise AttributeError("AI client not available")
            
        except Exception as e:
            self.logger.error(f"Error loading AI client: {e}")
            # Keep the existing ai_client, don't fall back to mock
    

    
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
        language = parameters.get('language', 'python')
        
        if not requirements:
            return {'error': 'Requirements not provided.'}
        
        try:
            if self.ai_client:
                # AI-based code generation
                ai_generated_code = self._ai_generate_code(requirements, component_type, security_level, language)
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
            template_code = self._generate_from_template(requirements, component_type, security_level, language)
            return {
                'code': template_code['code'],
                'metadata': template_code['metadata'],
                'security_features': template_code['security_features'],
                'ai_enhanced': False,
                'component_type': component_type,
                'security_level': security_level,
                'language': language
            }
            
        except Exception as e:
            self.logger.error(f"Code generation failed: {e}")
            return {'error': f'Code generation failed: {str(e)}'}
    
    def _ai_generate_code(self, requirements: str, component_type: str, security_level: str, language: str = "python") -> Dict[str, Any]:
        """AI-based code generation"""
        try:
            # Language-specific prompt generation
            if language.lower() in ["cpp", "c++"]:
                ai_prompt = self._generate_cpp_prompt(requirements, component_type, security_level)
            else:
                # Default to Python
                ai_prompt = self._generate_python_prompt(requirements, component_type, security_level)
            
            # Execute AI code generation
            if hasattr(self.ai_client, 'chat'):
                ai_response = self.ai_client.chat(ai_prompt)
            else:
                ai_response = self.ai_client.analyze_content(ai_prompt, "code_generation")
            
            # Handle both string and dict responses
            if isinstance(ai_response, str):
                # If AI returns a string, use it as code directly
                return {
                    'code': ai_response,
                    'metadata': {
                        'description': f'{language} ROS node',
                        'dependencies': ['rclcpp', 'std_msgs'] if language.lower() in ['cpp', 'c++'] else ['rclpy', 'std_msgs'],
                        'usage': 'Generated by AI',
                        'language': language
                    },
                    'security_features': ['Basic error handling', 'Input validation'],
                    'ai_enhanced': True,
                    'quality_score': self._assess_code_quality(ai_response)
                }
            elif isinstance(ai_response, dict) and 'code' in ai_response:
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
    
    def _generate_python_prompt(self, requirements: str, component_type: str, security_level: str) -> str:
        """Generate Python-specific AI prompt"""
        return f"""
        Generate high-quality, secure ROS 2 Python code according to the following requirements:
        
        Requirements: {requirements}
        Component type: {component_type}
        Security level: {security_level}
        
        Generate complete and secure Python code that includes:
        
        1. Required import statements (rclpy, std_msgs, sensor_msgs, etc.)
        2. Class definition with proper Node inheritance
        3. Secure initialization method with:
           - Proper QoS configuration
           - Parameter validation
           - Secure logging setup
        4. Essential security features:
           - Input validation with regex patterns for all received data
           - Comprehensive error handling with try-catch blocks
           - Secure parameter handling with validation callbacks
           - Safe file operations (if needed) with path validation
           - Appropriate logging without sensitive data exposure
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



    def _generate_cpp_prompt(self, requirements: str, component_type: str, security_level: str) -> str:
        """Generate C++-specific AI prompt"""
        return f"""
        Generate high-quality, secure ROS 2 C++ code according to the following requirements:
        
        Requirements: {requirements}
        Component type: {component_type}
        Security level: {security_level}
        
        Generate complete and secure C++ code that includes:
        
        1. Required header files (#include <rclcpp/rclcpp.hpp>, <std_msgs/msg/string.hpp>, etc.)
        2. Class definition inheriting from rclcpp::Node
        3. Secure initialization method with:
           - Proper QoS configuration
           - Parameter validation
           - Secure logging setup
        4. Essential security features:
           - Input validation for all received data
           - Comprehensive error handling with try-catch blocks
           - Secure parameter handling with validation callbacks
           - Safe file operations (if needed) with path validation
           - Appropriate logging without sensitive data exposure
           - Memory management (smart pointers, RAII)
        5. Main function and exception handling
        6. Detailed comments and documentation
        7. CMakeLists.txt and package.xml configuration
        
        Detailed requirements by security level:
        - low: Basic error handling, logging, memory safety
        - medium: Input validation + error handling + secure logging + basic authentication
        - high: All security features + encryption + multi-factor authentication + audit logs
        
        ROS 2 C++ security best practices:
        - Use smart pointers (std::shared_ptr, std::unique_ptr)
        - RAII for resource management
        - DDS security configuration
        - Topic encryption
        - Node namespace isolation
        - QoS security settings
        - Bounds checking for arrays/vectors
        
        Respond in the following JSON format:
        {{
            "code": "Complete C++ source code (.cpp file)",
            "header": "Header file (.hpp) if needed",
            "cmake": "CMakeLists.txt content",
            "package": "package.xml content", 
            "metadata": {{
                "description": "Detailed code description",
                "dependencies": ["List of required packages"],
                "usage": "Usage and compilation instructions",
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
        
        The code must be compilable and executable in a real ROS 2 environment and follow security best practices.
        """

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
            logging_method = r"""
    def secure_log(self, message: str, level: str = 'info') -> None:
        \"\"\"Secure logging (sensitive information masking)\"\"\"
        # Mask sensitive information
        import re
        masked_message = re.sub(r'password[=:]\s*\S+', 'password=***', message)
        masked_message = re.sub(r'api_key[=:]\s*\S+', 'api_key=***', masked_message)
        masked_message = re.sub(r'token[=:]\s*\S+', 'token=***', masked_message)
        
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
    
    def _generate_from_template(self, requirements: str, component_type: str, security_level: str, language: str = "python") -> Dict[str, Any]:
        """Template-based code generation - 단순화된 버전"""
        try:
            # 언어 결정
            lang = 'python' if language.lower() in ['python', 'py'] else 'cpp'
            
            # 기본 템플릿 가져오기
            base_template = self._get_template(lang, component_type)
            
            # 보안 기능 추가
            if security_level in ['medium', 'high']:
                base_template = self._add_security_to_template(base_template, lang, security_level)
            
            # 요구사항에 따른 커스터마이징
            customized_code = self._customize_template(base_template, requirements, lang)
            
            return {
                'code': customized_code,
                'metadata': {
                    'description': f'{component_type} based {lang} code',
                    'dependencies': ['rclpy', 'std_msgs'] if lang == 'python' else ['rclcpp', 'std_msgs'],
                    'usage': 'python3 generated_node.py' if lang == 'python' else 'colcon build && ros2 run <package_name> <node_name>',
                    'language': lang,
                    'file_extension': 'py' if lang == 'python' else 'cpp'
                },
                'security_features': self._get_security_features(security_level)
            }
            
        except Exception as e:
            self.logger.error(f"Template-based code generation failed: {e}")
            return {'error': f'Template code generation failed: {str(e)}'}
    
    def _get_template(self, language: str, component_type: str) -> str:
        """언어와 컴포넌트 타입에 따른 템플릿 반환"""
        if language == 'python':
            return self._get_python_template(component_type)
        else:
            return self._get_cpp_template(component_type)
    
    def _add_security_to_template(self, template: str, language: str, security_level: str) -> str:
        """템플릿에 보안 기능 추가"""
        security_code = ""
        
        # 기본 보안 기능 (medium 이상)
        if security_level in ['medium', 'high']:
            if language == 'python':
                security_code += self._get_python_security_pattern('input_validation')
                security_code += self._get_python_security_pattern('error_handling')
                security_code += self._get_python_security_pattern('secure_logging')
            else:
                security_code += self._get_cpp_security_pattern('input_validation')
                security_code += self._get_cpp_security_pattern('error_handling')
                security_code += self._get_cpp_security_pattern('secure_logging')
        
        # 고급 보안 기능 (high만)
        if security_level == 'high':
            if language == 'python':
                security_code += self._get_python_security_pattern('authentication')
                security_code += self._get_python_security_pattern('encryption')
            else:
                security_code += self._get_cpp_security_pattern('authentication')
                security_code += self._get_cpp_security_pattern('encryption')
        
        return template + security_code
    
    def _customize_template(self, template: str, requirements: Any, language: str) -> str:
        """템플릿을 요구사항에 맞게 커스터마이징"""
        customized = template
        
        # 요구사항 텍스트 추출
        if isinstance(requirements, dict):
            req_text = requirements.get('user_request', str(requirements))
        else:
            req_text = str(requirements)
        
        # 속도 제어 노드로 변경
        if 'speed' in req_text.lower() or 'velocity' in req_text.lower():
            if language == 'python':
                customized = customized.replace('GenericNode', 'SpeedControlNode')
                customized = customized.replace('generic_node', 'speed_control_node')
                customized = customized.replace('generic_topic', 'speed_command')
            else:  # C++
                customized = customized.replace('GenericNode', 'SpeedControlNode')
                customized = customized.replace('generic_node', 'speed_control_node')
                customized = customized.replace('generic_topic', 'speed_command')
                
                # Float64 메시지 타입으로 변경
                if '#include <std_msgs/msg/string.hpp>' in customized:
                    customized = customized.replace('#include <std_msgs/msg/string.hpp>', '#include <std_msgs/msg/float64.hpp>')
                if 'std_msgs::msg::String' in customized:
                    customized = customized.replace('std_msgs::msg::String', 'std_msgs::msg::Float64')
                if 'message.data = "Hello World"' in customized:
                    customized = customized.replace('message.data = "Hello World"', 'message.data = 0.0')
        
        return customized
    
    def _get_security_features(self, security_level: str) -> List[str]:
        """List of features by security level"""
        if security_level == 'low':
            return ['Basic error handling']
        elif security_level == 'medium':
            return ['Input validation', 'Error handling', 'Secure logging']
        else:  # high
            return ['Input validation', 'Error handling', 'Secure logging', 'Authentication', 'Encryption']
    
    def _get_python_security_patterns(self, security_level: str) -> str:
        """Python용 보안 패턴 반환"""
        patterns = ""
        
        if security_level in ['medium', 'high']:
            patterns += self.security_patterns['input_validation']
            patterns += "\n" + self.security_patterns['error_handling']
            patterns += "\n" + self.security_patterns['secure_logging']
        
        if security_level == 'high':
            patterns += "\n" + self.security_patterns['authentication']
            patterns += "\n" + self.security_patterns['encryption']
        
        return patterns
    
    def _get_cpp_security_patterns(self, security_level: str) -> str:
        """C++용 보안 패턴 반환"""
        patterns = ""
        
        if security_level in ['medium', 'high']:
            patterns += self._get_cpp_input_validation_pattern()
            patterns += "\n" + self._get_cpp_error_handling_pattern()
            patterns += "\n" + self._get_cpp_secure_logging_pattern()
        
        if security_level == 'high':
            patterns += "\n" + self._get_cpp_authentication_pattern()
            patterns += "\n" + self._get_cpp_encryption_pattern()
        
        return patterns
    
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
        return r'''
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
            'security_patterns': list(self.security_patterns.keys())
        }

    def generate_code(self, plan: str, security_guidelines: str, language: str = "python") -> str:
        """Generate ROS code based on plan and security guidelines"""
        try:
            # Use the proper code generation pipeline
            parameters = {
                'requirements': f"Plan: {plan}\nSecurity Guidelines: {security_guidelines}",
                'component_type': 'basic_node',
                'security_level': 'medium',
                'language': language
            }
            
            result = self._generate_ros_code(parameters)
            
            if 'error' in result:
                return f"# Code generation failed: {result['error']}"
            
            return result.get('code', '# No code generated')
            
        except Exception as e:
            self.logger.error(f"Code generation failed: {e}")
            return f"# Code generation failed: {str(e)}\n# Please use a basic template."

    def add_feedback_to_history(self, feedback: str):
        """Add feedback to generation history for improvement"""
        # Store feedback for next generation attempt
        if not hasattr(self, 'feedback_history'):
            self.feedback_history = []
        self.feedback_history.append(feedback)
        self.logger.info(f"Feedback added: {feedback[:100]}...")
    
    def set_ai_client(self, ai_client):
        """AI 클라이언트 설정"""
        self.llm_client = ai_client
        self.ai_client = ai_client
        self.logger.info(f"AI 클라이언트가 설정되었습니다: {type(ai_client).__name__}")

    def generate_ros_package(self, execution_spec: Dict[str, Any], output_dir: str = "ros_ws") -> Dict[str, Any]:
        """실행 명세를 바탕으로 ROS 패키지 생성"""
        try:
            language = execution_spec.get('node', {}).get('language', 'cpp')
            
            if language == 'cpp':
                return self._generate_cpp_package(execution_spec, output_dir)
            elif language == 'python':
                return self._generate_python_package(execution_spec, output_dir)
            else:
                raise ValueError(f"지원하지 않는 언어: {language}")
                
        except Exception as e:
            self.logger.error(f"ROS 패키지 생성 실패: {e}")
            return {"error": str(e)}

    def _generate_cpp_package(self, execution_spec: Dict[str, Any], output_dir: str) -> Dict[str, Any]:
        """C++ ROS 패키지 생성"""
        try:
            # 1. src/ 폴더에 .cpp 파일 생성
            cpp_code = self._generate_cpp_node_code(execution_spec)
            
            # 2. include/ 폴더에 .hpp 파일 생성
            hpp_code = self._generate_cpp_header_code(execution_spec)
            
            # 3. CMakeLists.txt 수정
            cmake_content = self._generate_cmake_content(execution_spec)
            
            # 4. package.xml 수정
            package_xml = self._generate_package_xml(execution_spec)
            
            # 파일 생성
            self._write_cpp_package_files(output_dir, execution_spec, cpp_code, hpp_code, cmake_content, package_xml)
            
            return {"status": "success", "language": "cpp"}
            
        except Exception as e:
            self.logger.error(f"C++ 패키지 생성 실패: {e}")
            return {"error": str(e)}

    def _generate_python_package(self, execution_spec: Dict[str, Any], output_dir: str) -> Dict[str, Any]:
        """Python ROS 패키지 생성"""
        try:
            # 1. ros_study_py/ 폴더에 .py 파일 생성
            python_code = self._generate_python_node_code(execution_spec)
            
            # 2. setup.py 수정
            setup_py = self._generate_setup_py(execution_spec)
            
            # 3. package.xml 수정
            package_xml = self._generate_package_xml(execution_spec)
            
            # 파일 생성
            self._write_python_package_files(output_dir, execution_spec, python_code, setup_py, package_xml)
            
            return {"status": "success", "language": "python"}
            
        except Exception as e:
            self.logger.error(f"Python 패키지 생성 실패: {e}")
            return {"error": str(e)}

    def _generate_cpp_node_code(self, execution_spec: Dict[str, Any]) -> str:
        """C++ 노드 코드 생성"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_node')
        
        # 기본 C++ 노드 템플릿 사용
        return self.cpp_templates.get('basic_node', '').replace('ros_node', node_name)

    def _generate_cpp_header_code(self, execution_spec: Dict[str, Any]) -> str:
        """C++ 헤더 파일 코드 생성"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_node')
        
        return f"""#ifndef {node_name.upper()}_HPP_
#define {node_name.upper()}_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class {node_name.capitalize()} : public rclcpp::Node
{{
public:
    {node_name.capitalize()}();
    ~{node_name.capitalize()}();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
}};

#endif // {node_name.upper()}_HPP_
"""

    def _generate_cmake_content(self, execution_spec: Dict[str, Any]) -> str:
        """CMakeLists.txt 내용 생성"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_node')
        
        return f"""cmake_minimum_required(VERSION 3.8)
project({node_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable({node_name} src/{node_name}.cpp)
ament_target_dependencies({node_name} rclcpp std_msgs)

install(TARGETS
  {node_name}
  DESTINATION lib/${{PROJECT_NAME}})

ament_package()
"""

    def _generate_package_xml(self, execution_spec: Dict[str, Any]) -> str:
        """package.xml 내용 생성"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_node')
        description = node_info.get('description', 'ROS Node')
        
        return f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{node_name}</name>
  <version>0.0.0</version>
  <description>{description}</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

    def _generate_python_node_code(self, execution_spec: Dict[str, Any]) -> str:
        """Python 노드 코드 생성"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_node')
        
        # 기본 Python 노드 템플릿 사용
        return self.python_templates.get('basic_node', '').replace('ros_node', node_name)

    def _generate_setup_py(self, execution_spec: Dict[str, Any]) -> str:
        """setup.py 내용 생성"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_node')
        description = node_info.get('description', 'ROS Node')
        
        return f"""from setuptools import setup

package_name = '{node_name}'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='{description}',
    license='TODO',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
            '{node_name} = {node_name}.{node_name}:main',
        ],
    }},
)
"""

    def _write_cpp_package_files(self, output_dir: str, execution_spec: Dict[str, Any], 
                                cpp_code: str, hpp_code: str, cmake_content: str, package_xml: str):
        """기존 C++ 패키지 구조에 내용 채우기"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_study_cpp')  # 기본값을 기존 패키지명으로
        
        # 기존 패키지 디렉토리 경로 (ros_ws/src/ros_study_cpp/)
        package_dir = os.path.join(output_dir, 'src', node_name)
        
        # 기존 디렉토리가 존재하는지 확인
        if not os.path.exists(package_dir):
            self.logger.warning(f"기존 패키지 디렉토리가 없음: {package_dir}")
            # 기존 구조가 없으면 새로 생성
            os.makedirs(os.path.join(package_dir, 'src'), exist_ok=True)
            os.makedirs(os.path.join(package_dir, 'include', node_name), exist_ok=True)
        
        # 기존 구조에 파일 내용만 채우기
        # 1. src/ 폴더에 .cpp 파일 생성 (기존 빈 폴더 활용)
        cpp_file_path = os.path.join(package_dir, 'src', f'{node_name}.cpp')
        with open(cpp_file_path, 'w', encoding='utf-8') as f:
            f.write(cpp_code)
        
        # 2. include/ 폴더에 .hpp 파일 생성 (기존 빈 폴더 활용)
        hpp_file_path = os.path.join(package_dir, 'include', node_name, f'{node_name}.hpp')
        with open(hpp_file_path, 'w', encoding='utf-8') as f:
            f.write(hpp_code)
        
        # 3. 기존 CMakeLists.txt 내용 교체
        cmake_file_path = os.path.join(package_dir, 'CMakeLists.txt')
        with open(cmake_file_path, 'w', encoding='utf-8') as f:
            f.write(cmake_content)
        
        # 4. 기존 package.xml 내용 교체
        package_xml_path = os.path.join(package_dir, 'package.xml')
        with open(package_xml_path, 'w', encoding='utf-8') as f:
            f.write(package_xml)
        
        self.logger.info(f"기존 C++ 패키지 구조에 내용 채우기 완료: {package_dir}")

    def _write_python_package_files(self, output_dir: str, execution_spec: Dict[str, Any], 
                                   python_code: str, setup_py: str, package_xml: str):
        """기존 Python 패키지 구조에 내용 채우기"""
        node_info = execution_spec.get('node', {})
        node_name = node_info.get('name', 'ros_study_py')  # 기본값을 기존 패키지명으로
        
        # 기존 패키지 디렉토리 경로 (ros_ws/src/ros_study_py/)
        package_dir = os.path.join(output_dir, 'src', node_name)
        
        # 기존 디렉토리가 존재하는지 확인
        if not os.path.exists(package_dir):
            self.logger.warning(f"기존 패키지 디렉토리가 없음: {package_dir}")
            # 기존 구조가 없으면 새로 생성
            os.makedirs(os.path.join(package_dir, node_name), exist_ok=True)
            os.makedirs(os.path.join(package_dir, 'resource'), exist_ok=True)
        
        # 기존 구조에 파일 내용만 채우기
        # 1. ros_study_py/ 폴더에 .py 파일 생성 (기존 빈 폴더 활용)
        python_file_path = os.path.join(package_dir, node_name, f'{node_name}.py')
        with open(python_file_path, 'w', encoding='utf-8') as f:
            f.write(python_code)
        
        # 2. 기존 setup.py 내용 교체
        setup_py_path = os.path.join(package_dir, 'setup.py')
        with open(setup_py_path, 'w', encoding='utf-8') as f:
            f.write(setup_py)
        
        # 3. 기존 package.xml 내용 교체
        package_xml_path = os.path.join(package_dir, 'package.xml')
        with open(package_xml_path, 'w', encoding='utf-8') as f:
            f.write(package_xml)
        
        # 4. __init__.py 파일 생성 (기존 빈 폴더 활용)
        init_file_path = os.path.join(package_dir, node_name, '__init__.py')
        with open(init_file_path, 'w', encoding='utf-8') as f:
            f.write('# ROS Python Package\n')
        
        # 5. resource 파일 생성 (기존 빈 폴더 활용)
        resource_file_path = os.path.join(package_dir, 'resource', node_name)
        with open(resource_file_path, 'w', encoding='utf-8') as f:
            f.write('')
        
        self.logger.info(f"기존 Python 패키지 구조에 내용 채우기 완료: {package_dir}")

    def generate_secure_ros_package(self, execution_spec: Dict[str, Any], security_guide: Dict[str, Any], output_dir: str = "ros_ws") -> Dict[str, Any]:
        """보안 가이드라인을 적용하여 안전한 ROS 패키지 생성"""
        try:
            # 1. 보안 검증
            if not self._validate_security_requirements(execution_spec, security_guide):
                return {"error": "보안 요구사항 검증 실패"}
            
            # 2. 보안 강화된 실행 명세 생성
            secure_spec = self._apply_security_enhancements(execution_spec, security_guide)
            
            # 3. ROS 패키지 생성
            result = self.generate_ros_package(secure_spec, output_dir)
            
            return result
            
        except Exception as e:
            self.logger.error(f"보안 ROS 패키지 생성 실패: {e}")
            return {"error": str(e)}

    def _validate_security_requirements(self, execution_spec: Dict[str, Any], security_guide: Dict[str, Any]) -> bool:
        """보안 요구사항 검증"""
        try:
            # 기본 보안 검증 로직
            required_security_features = security_guide.get('required_features', [])
            
            for feature in required_security_features:
                if feature not in execution_spec.get('security_features', []):
                    self.logger.warning(f"필수 보안 기능 누락: {feature}")
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"보안 요구사항 검증 실패: {e}")
            return False

    def _apply_security_enhancements(self, execution_spec: Dict[str, Any], security_guide: Dict[str, Any]) -> Dict[str, Any]:
        """보안 강화 적용"""
        try:
            enhanced_spec = execution_spec.copy()
            
            # 보안 기능 추가
            security_features = security_guide.get('security_features', {})
            enhanced_spec['security_features'] = security_features
            
            # 에러 처리 강화
            if 'error_handling' not in enhanced_spec:
                enhanced_spec['error_handling'] = []
            
            enhanced_spec['error_handling'].extend([
                "입력값 검증",
                "예외 처리",
                "로깅 보안"
            ])
            
            return enhanced_spec
            
        except Exception as e:
            self.logger.error(f"보안 강화 적용 실패: {e}")
            return execution_spec
    
    # C++ 보안 패턴 메서드들 추가
    def _get_cpp_input_validation_pattern(self) -> str:
        return '''
    // Input validation for C++
    bool validateInput(const std::string& input_data) {
        if (input_data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Input data is empty");
            return false;
        }
        
        // Length validation
        if (input_data.length() > 1000) {
            RCLCPP_ERROR(this->get_logger(), "Input data is too long");
            return false;
        }
        
        // Special character filtering
        if (input_data.find('<') != std::string::npos || 
            input_data.find('>') != std::string::npos ||
            input_data.find('"') != std::string::npos ||
            input_data.find('\'') != std::string::npos) {
            RCLCPP_ERROR(this->get_logger(), "Input data contains disallowed special characters");
            return false;
        }
        
        return true;
    }'''
    
    def _get_cpp_error_handling_pattern(self) -> str:
        return '''
    // Error handling for C++
    template<typename T>
    T safeExecute(std::function<T()> operation, T default_value = T{}) {
        try {
            return operation();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Operation execution failed: %s", e.what());
            return default_value;
        }
    }
    
    void handleCriticalError(const std::string& error) {
        RCLCPP_ERROR(this->get_logger(), "Critical error occurred: %s", error.c_str());
        // Transition to system safe state
        emergencyShutdown();
    }'''
    
    def _get_cpp_secure_logging_pattern(self) -> str:
        return '''
    // Secure logging for C++
    void secureLog(const std::string& message, const std::string& level = "info") {
        // Mask sensitive information
        std::string masked_message = message;
        
        // Mask passwords
        size_t pos = masked_message.find("password=");
        if (pos != std::string::npos) {
            size_t end_pos = masked_message.find_first_of(" \n", pos);
            if (end_pos != std::string::npos) {
                masked_message.replace(pos + 9, end_pos - pos - 9, "***");
            }
        }
        
        // Mask API keys
        pos = masked_message.find("api_key=");
        if (pos != std::string::npos) {
            size_t end_pos = masked_message.find_first_of(" \n", pos);
            if (end_pos != std::string::npos) {
                masked_message.replace(pos + 8, end_pos - pos - 8, "***");
            }
        }
        
        if (level == "info") {
            RCLCPP_INFO(this->get_logger(), "%s", masked_message.c_str());
        } else if (level == "warn") {
            RCLCPP_WARN(this->get_logger(), "%s", masked_message.c_str());
        } else if (level == "error") {
            RCLCPP_ERROR(this->get_logger(), "%s", masked_message.c_str());
        }
    }'''
    
    def _get_cpp_authentication_pattern(self) -> str:
        return '''
    // Authentication for C++
    bool authenticateUser(const std::string& credentials) {
        // TODO: Add authentication
        if (credentials.empty()) {
            return false;
        }
        
        // Implement authentication logic
        return true;
    }
    
    bool checkPermission(const std::string& user, const std::string& resource) {
        // TODO: Add permission check
        return true;
    }'''
    
    def _get_cpp_encryption_pattern(self) -> str:
        return '''
    // Encryption for C++
    std::string encryptData(const std::string& data) {
        // TODO: Add encryption
        // Simple hash for demonstration
        std::hash<std::string> hasher;
        return std::to_string(hasher(data));
    }
    
    std::string decryptData(const std::string& encrypted_data) {
        // TODO: Add decryption
        return encrypted_data;
    }'''
    
    def _extract_code_from_response(self, response: str) -> str:
        """AI 응답에서 코드 블록을 추출합니다."""
        try:
            # ```python, ```cpp, ```c++ 등의 코드 블록 찾기
            if "```python" in response:
                start_idx = response.find("```python") + 9
                end_idx = response.find("```", start_idx)
                if end_idx != -1:
                    return response[start_idx:end_idx].strip()
            elif "```cpp" in response:
                start_idx = response.find("```cpp") + 6
                end_idx = response.find("```", start_idx)
                if end_idx != -1:
                    return response[start_idx:end_idx].strip()
            elif "```c++" in response:
                start_idx = response.find("```c++") + 6
                end_idx = response.find("```", start_idx)
                if end_idx != -1:
                    return response[start_idx:end_idx].strip()
            elif "```" in response:
                # 일반적인 코드 블록
                start_idx = response.find("```") + 3
                end_idx = response.find("```", start_idx)
                if end_idx != -1:
                    return response[start_idx:end_idx].strip()
            
            # 코드 블록이 없으면 전체 응답 반환
            return response
            
        except Exception as e:
            self.logger.warning(f"코드 블록 추출 실패: {e}")
            return response
    
    # C++ ROS 노드 템플릿 메서드들 추가
    def _get_cpp_basic_node_template(self) -> str:
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class GenericNode : public rclcpp::Node
{
public:
    GenericNode() : Node("generic_node")
    {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("generic_topic", 10);
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GenericNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Generic node has been started");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}'''
    
    def _get_cpp_publisher_template(self) -> str:
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PublisherNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Publisher node has been started");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Published message";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}'''
    
    def _get_cpp_subscriber_template(self) -> str:
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        // Create subscription
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&SubscriberNode::listener_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscriber node has been started");
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}'''
    
    def _get_cpp_service_template(self) -> str:
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ServiceNode : public rclcpp::Node
{
public:
    ServiceNode() : Node("service_node")
    {
        // Create service
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_service", std::bind(&ServiceNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Service node has been started");
    }

private:
    void trigger_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Service called");
        response->success = true;
        response->message = "Service executed successfully";
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}'''
    
    def _get_cpp_parameter_template(self) -> str:
        return '''#include <rclcpp/rclcpp.hpp>

class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode() : Node("parameter_node")
    {
        // Declare parameter
        this->declare_parameter("my_parameter", "default_value");
        
        // Get parameter value
        std::string parameter_value = this->get_parameter("my_parameter").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter value: %s", parameter_value.c_str());
        
        RCLCPP_INFO(this->get_logger(), "Parameter node has been started");
    }

    std::string get_parameter_value()
    {
        return this->get_parameter("my_parameter").as_string();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}'''
    
    # === 통합된 보안 패턴 시스템 ===
    
    def _get_python_security_pattern(self, pattern_type: str) -> str:
        """Python 보안 패턴 반환"""
        patterns = {
            'input_validation': '''
    def validate_input(self, input_data: str) -> str:
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
        
        return input_data.strip()''',
            
            'error_handling': '''
    def safe_execute(self, operation, *args, **kwargs):
        """Safe operation execution"""
        try:
            result = operation(*args, **kwargs)
            return result
        except Exception as e:
            self.get_logger().error(f"Operation execution failed: {e}")
            return None
    
    def handle_critical_error(self, error):
        """Critical error handling"""
        self.get_logger().error(f"Critical error occurred: {error}")
        # Transition to system safe state
        self.emergency_shutdown()''',
            
            'secure_logging': r'''
    def secure_log(self, message: str, level: str = 'info') -> None:
        """Secure logging (sensitive information masking)"""
        # Mask sensitive information
        import re
        masked_message = re.sub(r'password[=:]\s*\S+', 'password=***', message)
        masked_message = re.sub(r'api_key[=:]\s*\S+', 'api_key=***', message)
        
        if level == 'info':
            self.get_logger().info(masked_message)
        elif level == 'warn':
            self.get_logger().warn(masked_message)
        elif level == 'error':
            self.get_logger().error(masked_message)''',
            
            'authentication': '''
    def authenticate_user(self, credentials):
        """User authentication"""
        # TODO: Add authentication
        if not credentials:
            return False
        return True
    
    def check_permission(self, user, resource):
        """Permission check"""
        # TODO: Add permission check
        return True''',
            
            'encryption': '''
    def encrypt_data(self, data):
        """Data encryption"""
        # TODO: Add encryption
        import hashlib
        return hashlib.sha256(data.encode()).hexdigest()
    
    def decrypt_data(self, encrypted_data):
        """Data decryption"""
        # TODO: Add decryption
        return encrypted_data'''
        }
        return patterns.get(pattern_type, '')
    
    def _get_cpp_security_pattern(self, pattern_type: str) -> str:
        """C++ 보안 패턴 반환"""
        patterns = {
            'input_validation': '''
    // Input validation for C++
    bool validateInput(const std::string& input_data) {
        if (input_data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Input data is empty");
            return false;
        }
        
        // Length validation
        if (input_data.length() > 1000) {
            RCLCPP_ERROR(this->get_logger(), "Input data is too long");
            return false;
        }
        
        // Special character filtering
        if (input_data.find('<') != std::string::npos || 
            input_data.find('>') != std::string::npos ||
            input_data.find('"') != std::string::npos ||
            input_data.find('\'') != std::string::npos) {
            RCLCPP_ERROR(this->get_logger(), "Input data contains disallowed special characters");
            return false;
        }
        
        return true;
    }''',
            
            'error_handling': '''
    // Error handling for C++
    template<typename T>
    T safeExecute(std::function<T()> operation, T default_value = T{}) {
        try {
            return operation();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Operation execution failed: %s", e.what());
            return default_value;
        }
    }
    
    void handleCriticalError(const std::string& error) {
        RCLCPP_ERROR(this->get_logger(), "Critical error occurred: %s", error.c_str());
        // Transition to system safe state
        emergencyShutdown();
    }''',
            
            'secure_logging': '''
    // Secure logging for C++
    void secureLog(const std::string& message, const std::string& level = "info") {
        // Mask sensitive information
        std::string masked_message = message;
        
        // Mask passwords
        size_t pos = masked_message.find("password=");
        if (pos != std::string::npos) {
            size_t end_pos = masked_message.find_first_of(" \n", pos);
            if (end_pos != std::string::npos) {
                masked_message.replace(pos + 9, end_pos - pos - 9, "***");
            }
        }
        
        // Mask API keys
        pos = masked_message.find("api_key=");
        if (pos != std::string::npos) {
            size_t end_pos = masked_message.find_first_of(" \n", pos);
            if (end_pos != std::string::npos) {
                masked_message.replace(pos + 8, end_pos - pos - 8, "***");
            }
        }
        
        if (level == "info") {
            RCLCPP_INFO(this->get_logger(), "%s", masked_message.c_str());
        } else if (level == "warn") {
            RCLCPP_WARN(this->get_logger(), "%s", masked_message.c_str());
        } else if (level == "error") {
            RCLCPP_ERROR(this->get_logger(), "%s", masked_message.c_str());
        }
    }''',
            
            'authentication': '''
    // Authentication for C++
    bool authenticateUser(const std::string& credentials) {
        // TODO: Add authentication
        if (credentials.empty()) {
            return false;
        }
        return true;
    }
    
    bool checkPermission(const std::string& user, const std::string& resource) {
        // TODO: Add permission check
        return true;
    }''',
            
            'encryption': '''
    // Encryption for C++
    std::string encryptData(const std::string& data) {
        // TODO: Add encryption
        // Simple hash for demonstration
        std::hash<std::string> hasher;
        return std::to_string(hasher(data));
    }
    
    std::string decryptData(const std::string& encrypted_data) {
        // TODO: Add decryption
        return encrypted_data;
    }'''
        }
        return patterns.get(pattern_type, '')
    
    def _get_python_template(self, template_type: str) -> str:
        """Python 템플릿 반환"""
        templates = {
            'basic_node': '''#!/usr/bin/env python3
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
    main()''',
            
            'publisher': '''#!/usr/bin/env python3
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
    main()''',
            
            'subscriber': '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
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
    main()''',
            
            'service': '''#!/usr/bin/env python3
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
    main()''',
            
            'action': '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci', self.execute_callback)
        self.get_logger().info('Action node has been started')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
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
    main()''',
            
            'parameter': '''#!/usr/bin/env python3
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
        }
        return templates.get(template_type, templates['basic_node'])
    
    def _get_cpp_template(self, template_type: str) -> str:
        """C++ 템플릿 반환"""
        templates = {
            'basic_node': '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class GenericNode : public rclcpp::Node
{
public:
    GenericNode() : Node("generic_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("generic_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GenericNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Generic node has been started");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}''',
            
            'publisher': '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PublisherNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Publisher node has been started");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Published message";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}''',
            
            'subscriber': '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&SubscriberNode::listener_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscriber node has been started");
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}''',
            
            'service': '''#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ServiceNode : public rclcpp::Node
{
public:
    ServiceNode() : Node("service_node")
    {
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_service", std::bind(&ServiceNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service node has been started");
    }

private:
    void trigger_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Service called");
        response->success = true;
        response->message = "Service executed successfully";
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}''',
            
            'parameter': '''#include <rclcpp/rclcpp.hpp>

class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode() : Node("parameter_node")
    {
        this->declare_parameter("my_parameter", "default_value");
        std::string parameter_value = this->get_parameter("my_parameter").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter value: %s", parameter_value.c_str());
        RCLCPP_INFO(this->get_logger(), "Parameter node has been started");
    }

    std::string get_parameter_value()
    {
        return this->get_parameter("my_parameter").as_string();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}'''
        }
        return templates.get(template_type, templates['basic_node'])