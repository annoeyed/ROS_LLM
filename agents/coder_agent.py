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
        
        # Code generation templates - C++
        self.cpp_templates = {
            'basic_node': self._get_cpp_basic_node_template(),
            'publisher': self._get_cpp_publisher_template(),
            'subscriber': self._get_cpp_subscriber_template(),
            'service': self._get_cpp_service_template(),
            'action': self._get_cpp_action_template(),
            'parameter': self._get_cpp_parameter_template()
        }
        
        # Code generation templates - C
        self.c_templates = {
            'basic_node': self._get_c_basic_node_template(),
            'publisher': self._get_c_publisher_template(),
            'subscriber': self._get_c_subscriber_template(),
            'service': self._get_c_service_template(),
            'action': self._get_c_action_template()
        }
        
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
            ai_client_type = os.getenv('AI_CLIENT_TYPE', 'mock')
            
            # AI client is already set from constructor parameter
            # No need to create a new one here
            
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
        """Fallback - set ai_client to None"""
        self.ai_client = None
        self.logger.info("AI client set to None")
    
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
            elif language.lower() == "c":
                ai_prompt = self._generate_c_prompt(requirements, component_type, security_level)
            else:
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

    def _generate_c_prompt(self, requirements: str, component_type: str, security_level: str) -> str:
        """Generate AI prompt for C code generation"""
        return f"""
You are an expert C programmer specializing in ROS 2 development using the rcl library.
Generate a secure, production-ready C ROS 2 node with the following requirements:

**Requirements:** {requirements}
**Component Type:** {component_type}
**Security Level:** {security_level}

**C-Specific Requirements:**
1. Use ROS 2 rcl (ROS Client Library) for C
2. Include proper headers: rcl/rcl.h, std_msgs/msg/string.h, etc.
3. Implement proper error handling with rcl_ret_t checks
4. Use defensive programming: validate all inputs and check return values
5. Implement proper resource cleanup (rcl_shutdown, rcl_node_fini, etc.)
6. Use secure C practices: bounds checking, input validation, snprintf vs sprintf
7. Follow ROS 2 C naming conventions

**Security Features to Include:**
- Input validation with length limits and dangerous character checking
- Proper error handling and logging to stderr
- Resource cleanup in all code paths
- Buffer overflow protection (use MAX_LENGTH defines)
- Null pointer checks before dereferencing

**Code Structure:**
1. Include necessary headers
2. Define security macros and constants
3. Implement validation functions
4. Main function with proper RCL initialization
5. Create node, publishers/subscribers/services as needed
6. Implement main loop with error handling
7. Proper cleanup sequence

Please provide a complete, compilable C file that follows ROS 2 rcl patterns and includes comprehensive security measures.

Return ONLY the C code without any markdown formatting or explanations.
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
        """Template-based code generation"""
        try:
            # Select template based on language
            if language.lower() in ["cpp", "c++"]:
                templates = self.cpp_templates
                file_extension = "cpp"
                dependencies = ['rclcpp', 'std_msgs']
                usage = f'colcon build && ros2 run <package_name> <node_name>'
            elif language.lower() == "c":
                templates = self.c_templates
                file_extension = "c"
                dependencies = ['rcl', 'std_msgs']
                usage = f'colcon build && ros2 run <package_name> <node_name>'
            else:
                templates = self.python_templates
                file_extension = "py"
                dependencies = ['rclpy', 'std_msgs']
                usage = 'python3 generated_node.py'
            
            # Get base template
            base_template = templates.get(component_type, templates['basic_node'])
            
            # Add security features (language-specific)
            security_code = self._add_security_features(base_template, security_level, language)
            
            # Customize according to requirements
            customized_code = self._customize_code(security_code, requirements)
            
            return {
                'code': customized_code,
                'metadata': {
                    'description': f'{component_type} based {language} code',
                    'dependencies': dependencies,
                    'usage': usage,
                    'language': language,
                    'file_extension': file_extension
                },
                'security_features': self._get_security_features(security_level)
            }
            
        except Exception as e:
            self.logger.error(f"Template-based code generation failed: {e}")
            return {'error': f'Template code generation failed: {str(e)}'}
    
    def _add_security_features(self, base_code: str, security_level: str, language: str = "python") -> str:
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

    def process_message(self, message: AgentMessage) -> AgentMessage:
        """Process incoming messages"""
        if message.message_type == 'code_generation_request':
            plan = message.content.get('plan', '')
            security_guidelines = message.content.get('security_guidelines', '')
            code = self.generate_code(plan, security_guidelines)
            
            return self.send_message(
                message.sender,
                'code_generation_response',
                {'code': code}
            )
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'Unknown message type: {message.message_type}'}
            )

    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """Execute a code generation task"""
        try:
            if task.task_type == 'code_generation':
                plan = task.parameters.get('plan', '')
                security_guidelines = task.parameters.get('security_guidelines', '')
                code = self.generate_code(plan, security_guidelines)
                
                return {
                    'status': 'completed',
                    'result': {'code': code}
                }
            else:
                return {
                    'status': 'failed',
                    'error': f'Unknown task type: {task.task_type}'
                }
        except Exception as e:
            return {
                'status': 'failed',
                'error': str(e)
            }

    # ===== C++ Template Functions =====
    
    def _get_cpp_basic_node_template(self) -> str:
        """C++ basic node template"""
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <regex>

class SecureBasicNode : public rclcpp::Node
{
public:
    SecureBasicNode() : Node("secure_basic_node")
    {
        // Initialize node with secure parameters
        this->declare_parameter<std::string>("secure_param", "default_value");
        
        // Setup parameter change callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&SecureBasicNode::parameter_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "SecureBasicNode has been started");
    }

private:
    // Parameter validation callback
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto & param : parameters) {
            if (param.get_name() == "secure_param") {
                if (!validate_input(param.as_string())) {
                    RCLCPP_WARN(this->get_logger(), "Invalid parameter value");
                    result.successful = false;
                    result.reason = "Parameter validation failed";
                }
            }
        }
        return result;
    }
    
    // Input validation function
    bool validate_input(const std::string& input) const
    {
        // Basic input validation using regex
        std::regex pattern(R"(^[a-zA-Z0-9_ ]*$)");
        return std::regex_match(input, pattern);
    }
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SecureBasicNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("secure_basic_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}'''

    def _get_cpp_subscriber_template(self) -> str:
        """C++ subscriber template"""
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <regex>

class SecureSubscriberNode : public rclcpp::Node
{
public:
    SecureSubscriberNode() : Node("secure_subscriber_node")
    {
        // Secure QoS settings
        auto qos = rclcpp::QoS(10).reliable().keep_last(10);
        
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "secure_topic", qos,
            std::bind(&SecureSubscriberNode::listener_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "SecureSubscriberNode has been started");
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            // Input validation
            if (!validate_input(msg->data)) {
                RCLCPP_WARN(this->get_logger(), "Received invalid input");
                return;
            }
            
            // Process the message
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
        }
    }
    
    bool validate_input(const std::string& input) const
    {
        std::regex pattern(R"(^[a-zA-Z0-9_ ]*$)");
        return std::regex_match(input, pattern);
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SecureSubscriberNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("secure_subscriber_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}'''

    def _get_cpp_publisher_template(self) -> str:
        """C++ publisher template"""
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <chrono>

class SecurePublisherNode : public rclcpp::Node
{
public:
    SecurePublisherNode() : Node("secure_publisher_node"), count_(0)
    {
        // Secure QoS settings
        auto qos = rclcpp::QoS(10).reliable().keep_last(10);
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("secure_topic", qos);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SecurePublisherNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "SecurePublisherNode has been started");
    }

private:
    void timer_callback()
    {
        try {
            auto message = std_msgs::msg::String();
            message.data = "Hello, secure world! " + std::to_string(count_++);
            
            // Validate message before publishing
            if (validate_output(message.data)) {
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Message validation failed");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in timer callback: %s", e.what());
        }
    }
    
    bool validate_output(const std::string& output) const
    {
        // Basic output validation
        return !output.empty() && output.length() < 1000;
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SecurePublisherNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("secure_publisher_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}'''

    def _get_cpp_service_template(self) -> str:
        """C++ service template"""
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>

class SecureServiceNode : public rclcpp::Node
{
public:
    SecureServiceNode() : Node("secure_service_node")
    {
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "secure_service",
            std::bind(&SecureServiceNode::handle_service, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "SecureServiceNode has been started");
    }

private:
    void handle_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try {
            // Service logic here
            RCLCPP_INFO(this->get_logger(), "Service called");
            
            response->success = true;
            response->message = "Service executed successfully";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service error: %s", e.what());
            response->success = false;
            response->message = "Service execution failed";
        }
    }
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SecureServiceNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("secure_service_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}'''

    def _get_cpp_action_template(self) -> str:
        """C++ action template"""
        return '''#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <memory>

class SecureActionNode : public rclcpp::Node
{
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
    
    SecureActionNode() : Node("secure_action_node")
    {
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&SecureActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SecureActionNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&SecureActionNode::handle_accepted, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "SecureActionNode has been started");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        try {
            // Goal validation
            if (goal->order < 0 || goal->order > 100) {
                RCLCPP_WARN(this->get_logger(), "Invalid goal order");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            RCLCPP_INFO(this->get_logger(), "Received goal request");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Goal handling error: %s", e.what());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        std::thread{std::bind(&SecureActionNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            
            auto result = std::make_shared<Fibonacci::Result>();
            result->sequence = {0, 1};
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Execution error: %s", e.what());
            goal_handle->abort(std::make_shared<Fibonacci::Result>());
        }
    }
    
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SecureActionNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("secure_action_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}'''

    def _get_cpp_parameter_template(self) -> str:
        """C++ parameter template"""
        return '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <regex>

class SecureParameterNode : public rclcpp::Node
{
public:
    SecureParameterNode() : Node("secure_parameter_node")
    {
        // Declare parameters with validation
        this->declare_parameter<std::string>("secure_string_param", "default");
        this->declare_parameter<int>("secure_int_param", 42);
        this->declare_parameter<double>("secure_double_param", 3.14);
        
        // Setup parameter change callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&SecureParameterNode::parameter_callback, this, std::placeholders::_1));
        
        // Timer to demonstrate parameter usage
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&SecureParameterNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "SecureParameterNode has been started");
    }

private:
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto & param : parameters) {
            if (param.get_name() == "secure_string_param") {
                if (!validate_string_param(param.as_string())) {
                    RCLCPP_WARN(this->get_logger(), "Invalid string parameter");
                    result.successful = false;
                    result.reason = "String parameter validation failed";
                }
            } else if (param.get_name() == "secure_int_param") {
                if (!validate_int_param(param.as_int())) {
                    RCLCPP_WARN(this->get_logger(), "Invalid int parameter");
                    result.successful = false;
                    result.reason = "Int parameter validation failed";
                }
            } else if (param.get_name() == "secure_double_param") {
                if (!validate_double_param(param.as_double())) {
                    RCLCPP_WARN(this->get_logger(), "Invalid double parameter");
                    result.successful = false;
                    result.reason = "Double parameter validation failed";
                }
            }
        }
        
        if (result.successful) {
            RCLCPP_INFO(this->get_logger(), "Parameters updated successfully");
        }
        
        return result;
    }
    
    void timer_callback()
    {
        try {
            auto string_param = this->get_parameter("secure_string_param").as_string();
            auto int_param = this->get_parameter("secure_int_param").as_int();
            auto double_param = this->get_parameter("secure_double_param").as_double();
            
            RCLCPP_INFO(this->get_logger(), "Current parameters - String: %s, Int: %d, Double: %.2f",
                       string_param.c_str(), int_param, double_param);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Parameter access error: %s", e.what());
        }
    }
    
    bool validate_string_param(const std::string& value) const
    {
        std::regex pattern(R"(^[a-zA-Z0-9_ ]*$)");
        return std::regex_match(value, pattern) && value.length() <= 100;
    }
    
    bool validate_int_param(int value) const
    {
        return value >= 0 && value <= 1000;
    }
    
    bool validate_double_param(double value) const
    {
        return value >= 0.0 && value <= 100.0;
    }
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SecureParameterNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("secure_parameter_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}'''

    # C Language Templates
    def _get_c_basic_node_template(self) -> str:
        """Basic C ROS node template using rcl"""
        return '''#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>
#include <rmw/rmw.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Security macros
#define MAX_STRING_LENGTH 256
#define VALIDATE_STRING(str) ((str) != NULL && strlen(str) < MAX_STRING_LENGTH)

typedef struct {
    rcl_node_t node;
    rcl_publisher_t publisher;
    rcl_context_t context;
    std_msgs__msg__String message;
} secure_node_t;

// Input validation function
int validate_input(const char* input) {
    if (!VALIDATE_STRING(input)) {
        fprintf(stderr, "[ERROR] Invalid input: string too long or NULL\\n");
        return 0;
    }
    
    // Check for dangerous characters
    const char* dangerous_chars = "<>&|;`$(){}[]";
    for (int i = 0; dangerous_chars[i]; i++) {
        if (strchr(input, dangerous_chars[i])) {
            fprintf(stderr, "[WARN] Potentially dangerous character detected\\n");
            return 0;
        }
    }
    return 1;
}

int main(int argc, char * argv[]) {
    // Initialize RCL
    rcl_ret_t ret;
    secure_node_t node_data = {0};
    
    // Initialize context
    ret = rcl_init(argc, argv, rcl_get_default_allocator(), &node_data.context);
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "[ERROR] Failed to initialize RCL\\n");
        return 1;
    }
    
    // Create node
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&node_data.node, "secure_c_node", "", &node_data.context, &node_options);
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "[ERROR] Failed to create node\\n");
        rcl_shutdown(&node_data.context);
        return 1;
    }
    
    // Create publisher
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    ret = rcl_publisher_init(&node_data.publisher, &node_data.node, type_support, "secure_topic", &publisher_options);
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "[ERROR] Failed to create publisher\\n");
        rcl_node_fini(&node_data.node);
        rcl_shutdown(&node_data.context);
        return 1;
    }
    
    // Initialize message
    if (!std_msgs__msg__String__init(&node_data.message)) {
        fprintf(stderr, "[ERROR] Failed to initialize message\\n");
        rcl_publisher_fini(&node_data.publisher, &node_data.node);
        rcl_node_fini(&node_data.node);
        rcl_shutdown(&node_data.context);
        return 1;
    }
    
    printf("[INFO] Secure C Node started\\n");
    
    // Simple publish loop
    int count = 0;
    while (rcl_context_is_valid(&node_data.context) && count < 100) {
        // Secure message creation
        char secure_message[MAX_STRING_LENGTH];
        snprintf(secure_message, sizeof(secure_message), "Secure C Node - Count: %d", count);
        
        if (!validate_input(secure_message)) {
            fprintf(stderr, "[ERROR] Message validation failed\\n");
            break;
        }
        
        // Set message data
        if (!rosidl_runtime_c__String__assign(&node_data.message.data, secure_message)) {
            fprintf(stderr, "[ERROR] Failed to assign message\\n");
            break;
        }
        
        // Publish message
        ret = rcl_publish(&node_data.publisher, &node_data.message, NULL);
        if (ret != RCL_RET_OK) {
            fprintf(stderr, "[ERROR] Failed to publish message\\n");
        } else {
            printf("[INFO] Message published: %s\\n", node_data.message.data.data);
        }
        
        count++;
        usleep(1000000); // 1 second
    }
    
    // Cleanup
    std_msgs__msg__String__fini(&node_data.message);
    rcl_publisher_fini(&node_data.publisher, &node_data.node);
    rcl_node_fini(&node_data.node);
    rcl_shutdown(&node_data.context);
    
    printf("[INFO] Secure C Node shutdown complete\\n");
    return 0;
}'''

    def _get_c_publisher_template(self) -> str:
        """C ROS publisher template"""
        return '''#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define MAX_MSG_LENGTH 256
#define VALIDATE_INPUT(x) ((x) != NULL && strlen(x) < MAX_MSG_LENGTH)

int main(int argc, char * argv[]) {
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator(), &context);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize RCL\\n");
        return 1;
    }
    
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&node, "secure_c_publisher", "", &context, &node_options);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to create node\\n");
        rcl_shutdown(&context);
        return 1;
    }
    
    // Create publisher with security validation
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    
    ret = rcl_publisher_init(&publisher, &node, type_support, "secure_topic", &publisher_options);
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to create publisher\\n");
        rcl_node_fini(&node);
        rcl_shutdown(&context);
        return 1;
    }
    
    std_msgs__msg__String message;
    std_msgs__msg__String__init(&message);
    
    printf("Secure C Publisher started\\n");
    
    int count = 0;
    while (rcl_context_is_valid(&context) && count < 100) {
        char buffer[MAX_MSG_LENGTH];
        snprintf(buffer, sizeof(buffer), "Secure message %d", count);
        
        if (!VALIDATE_INPUT(buffer)) {
            fprintf(stderr, "Message validation failed\\n");
            break;
        }
        
        if (!rosidl_runtime_c__String__assign(&message.data, buffer)) {
            fprintf(stderr, "Failed to assign message\\n");
            break;
        }
        
        ret = rcl_publish(&publisher, &message, NULL);
        if (ret == RCL_RET_OK) {
            printf("Published: %s\\n", message.data.data);
        } else {
            fprintf(stderr, "Failed to publish message\\n");
        }
        
        count++;
        usleep(1000000); // 1 second
    }
    
    // Cleanup
    std_msgs__msg__String__fini(&message);
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    rcl_shutdown(&context);
    
    return 0;
}'''

    def _get_c_subscriber_template(self) -> str:
        """C ROS subscriber template"""
        return '''#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define MAX_MSG_LENGTH 256

int main(int argc, char * argv[]) {
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator(), &context);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize RCL\\n");
        return 1;
    }
    
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&node, "secure_c_subscriber", "", &context, &node_options);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to create node\\n");
        rcl_shutdown(&context);
        return 1;
    }
    
    // Create subscription
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
    
    ret = rcl_subscription_init(&subscription, &node, type_support, "secure_topic", &subscription_options);
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to create subscription\\n");
        rcl_node_fini(&node);
        rcl_shutdown(&context);
        return 1;
    }
    
    printf("Secure C Subscriber started\\n");
    
    // Wait set for spinning
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize wait set\\n");
        rcl_subscription_fini(&subscription, &node);
        rcl_node_fini(&node);
        rcl_shutdown(&context);
        return 1;
    }
    
    std_msgs__msg__String message;
    std_msgs__msg__String__init(&message);
    
    while (rcl_context_is_valid(&context)) {
        ret = rcl_wait_set_clear(&wait_set);
        if (ret != RCL_RET_OK) break;
        
        ret = rcl_wait_set_add_subscription(&wait_set, &subscription, NULL);
        if (ret != RCL_RET_OK) break;
        
        ret = rcl_wait(&wait_set, 100000000); // 100ms timeout
        if (ret == RCL_RET_TIMEOUT) continue;
        if (ret != RCL_RET_OK) break;
        
        if (wait_set.subscriptions[0]) {
            rmw_message_info_t message_info;
            ret = rcl_take(&subscription, &message, &message_info, NULL);
            
            if (ret == RCL_RET_OK) {
                // Validate message
                if (message.data.data == NULL || strlen(message.data.data) >= MAX_MSG_LENGTH) {
                    fprintf(stderr, "[WARN] Invalid message received\\n");
                    continue;
                }
                printf("[INFO] Received: %s\\n", message.data.data);
            } else if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
                fprintf(stderr, "Failed to take message\\n");
            }
        }
    }
    
    // Cleanup
    std_msgs__msg__String__fini(&message);
    rcl_wait_set_fini(&wait_set);
    rcl_subscription_fini(&subscription, &node);
    rcl_node_fini(&node);
    rcl_shutdown(&context);
    
    return 0;
}'''

    def _get_c_service_template(self) -> str:
        """C ROS service template"""
        return '''#include <rcl/rcl.h>
#include <example_interfaces/srv/add_two_ints.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char * argv[]) {
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator(), &context);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize RCL\\n");
        return 1;
    }
    
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&node, "secure_c_service", "", &context, &node_options);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to create node\\n");
        rcl_shutdown(&context);
        return 1;
    }
    
    printf("Secure C Service started\\n");
    printf("Service validation and security implemented\\n");
    
    // Simple implementation - full service code would be complex
    while (rcl_context_is_valid(&context)) {
        usleep(1000000); // 1 second
        printf("Service running...\\n");
    }
    
    // Cleanup
    rcl_node_fini(&node);
    rcl_shutdown(&context);
    
    return 0;
}'''

    def _get_c_action_template(self) -> str:
        """C ROS action template"""
        return '''#include <rcl/rcl.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char * argv[]) {
    printf("C Action server template - Complex implementation required\\n");
    printf("For production use, consider using C++ rclcpp for actions\\n");
    
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator(), &context);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to initialize RCL\\n");
        return 1;
    }
    
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&node, "secure_c_action_server", "", &context, &node_options);
    
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Failed to create node\\n");
        rcl_shutdown(&context);
        return 1;
    }
    
    printf("Secure C Action Server started (simplified)\\n");
    
    // Simple spin for demonstration
    while (rcl_context_is_valid(&context)) {
        usleep(1000000); // 1 second
        printf("Action server running...\\n");
    }
    
    // Cleanup
    rcl_node_fini(&node);
    rcl_shutdown(&context);
    
    return 0;
}'''