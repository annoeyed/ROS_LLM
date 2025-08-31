from .base_agent import BaseAgent, AgentMessage, AgentTask
from typing import Dict, Any
import subprocess
import tempfile
import os
import sys

class JudgeAgent(BaseAgent):
    """
    LLM as Judge Agent: Evaluates the generated code for security, functionality, and compilation.
    """
    def __init__(self, llm_client):
        """
        Initializes the JudgeAgent with a language model client.

        Args:
            llm_client: The client for interacting with the language model.
        """
        super().__init__(agent_id="judge_agent", agent_name="JudgeAgent")
        self.llm_client = llm_client

    def run(self, code_and_context: str) -> str:
        """
        Runs the agent to evaluate the security, functionality, and compilation of the generated code.

        Args:
            code_and_context: A string containing the user instruction, security guidelines,
                              and the generated code to be evaluated.

        Returns:
            A string containing the comprehensive evaluation and judgment.
        """
        # Enhanced prompt for comprehensive evaluation
        prompt = f"""
You are an expert ROS code auditor and security specialist. Your task is to perform a COMPREHENSIVE evaluation of the provided ROS code covering:

1. **SECURITY ANALYSIS** - Check for vulnerabilities, input validation, error handling
2. **FUNCTIONALITY VERIFICATION** - Ensure the code meets the specified requirements
3. **CODE QUALITY** - Check for proper ROS patterns, error handling, logging
4. **COMPILATION READINESS** - Verify syntax, imports, and basic structure

### Context and Code to Evaluate
{code_and_context}

### Comprehensive Evaluation Criteria

**SECURITY REQUIREMENTS (MUST PASS ALL):**
- Input validation and sanitization
- Proper error handling with try-catch blocks
- Secure logging without sensitive data exposure
- No obvious security vulnerabilities (CWE-20, CWE-78, CWE-22, CWE-73)
- ROS-specific security: secure QoS settings, parameter validation

**FUNCTIONALITY REQUIREMENTS (MUST PASS ALL):**
- Code implements the requested ROS functionality
- Proper ROS node structure and lifecycle management
- Correct topic/service/action usage
- Appropriate error handling for ROS operations

**CODE QUALITY REQUIREMENTS (MUST PASS ALL):**
- Clean, readable code structure
- Proper imports and dependencies
- Consistent coding style
- Meaningful variable names and comments

**COMPILATION REQUIREMENTS (MUST PASS ALL):**
- Valid Python/C++ syntax
- All imports are available and correct
- No undefined variables or functions
- Proper indentation and structure

### Final Judgment Categories:
- **"SECURE & FUNCTIONAL"** - Passes ALL security, functionality, and compilation checks
- **"NEEDS IMPROVEMENT"** - Fails some checks but fixable
- **"CRITICAL ISSUES"** - Fails critical security or compilation checks

Provide detailed feedback for each category and specific recommendations for improvement.
"""
        # Generate the evaluation using the language model.
        try:
            if hasattr(self.llm_client, 'chat'):
                response = self.llm_client.chat(prompt)
            elif hasattr(self.llm_client, 'generate_response'):
                response = self.llm_client.generate_response(prompt)
            else:
                raise AttributeError("AI 클라이언트가 chat 또는 generate_response 메서드를 지원하지 않습니다.")
            return response
        except Exception as e:
            error_msg = f"AI 클라이언트 호출 실패: {str(e)}"
            print(f"[ERROR] {error_msg}")
            return f"코드 평가를 수행할 수 없습니다. {error_msg}"

    def set_ai_client(self, ai_client):
        """AI 클라이언트 설정"""
        self.llm_client = ai_client
        self.logger.info(f"AI 클라이언트가 설정되었습니다: {type(ai_client).__name__}")

    def process_message(self, message: AgentMessage) -> AgentMessage:
        """Process incoming messages"""
        if message.message_type == 'code_evaluation_request':
            code_and_context = message.content.get('code_and_context', '')
            evaluation = self.run(code_and_context)
            
            return self.send_message(
                message.sender,
                'code_evaluation_response',
                {'evaluation': evaluation}
            )
        else:
            return self.send_message(
                message.sender,
                'error',
                {'error': f'Unknown message type: {message.message_type}'}
            )

    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """Execute a code evaluation task"""
        try:
            if task.task_type == 'code_evaluation':
                code_and_context = task.parameters.get('code_and_context', '')
                evaluation = self.run(code_and_context)
                
                return {
                    'status': 'completed',
                    'result': {'evaluation': evaluation}
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

    def verify_code(self, generated_code: str, security_guidelines: str) -> tuple[bool, str]:
        """
        Comprehensive verification of generated code covering security, functionality, and compilation.
        
        Args:
            generated_code: The code to be verified
            security_guidelines: The security guidelines to check against
            
        Returns:
            Tuple of (is_safe: bool, feedback: str)
        """
        # Enhanced security guidelines
        comprehensive_guidelines = """Comprehensive Security & Quality Requirements:

**CRITICAL SECURITY (MUST PASS):**
- Input validation (CWE-20, CWE-78, CWE-22)
- Error handling with try-catch blocks (CWE-544, CWE-778)
- Secure logging without sensitive data (CWE-532)
- File operation security (CWE-73, CWE-377)
- ROS-specific: secure QoS, parameter validation, topic validation

**FUNCTIONALITY REQUIREMENTS (MUST PASS):**
- Implements requested ROS functionality
- Proper ROS node structure
- Correct topic/service/action usage
- Appropriate error handling for ROS operations

**CODE QUALITY (MUST PASS):**
- Clean, readable structure
- Proper imports and dependencies
- Consistent coding style
- Meaningful names and comments

**COMPILATION READINESS (MUST PASS):**
- Valid syntax
- All imports available
- No undefined variables/functions
- Proper structure and indentation
"""
        
        code_and_context = f"""
### Comprehensive Guidelines:
{comprehensive_guidelines}

### Generated Code to Evaluate:
{generated_code}

### Strict Evaluation Instructions:
This code must pass ALL categories to be considered secure and functional:

1. **SECURITY CHECK** - Must have input validation, error handling, secure logging
2. **FUNCTIONALITY CHECK** - Must implement requested ROS functionality correctly
3. **QUALITY CHECK** - Must be clean, readable, and well-structured
4. **COMPILATION CHECK** - Must have valid syntax and structure

**ONLY JUDGE AS "SECURE & FUNCTIONAL" if ALL requirements are met.**
**JUDGE AS "NEEDS IMPROVEMENT" if some requirements are met but fixable.**
**JUDGE AS "CRITICAL ISSUES" if critical security or compilation issues exist.**
"""
        
        # Get LLM evaluation
        evaluation = self.run(code_and_context)
        
        # Perform additional compilation check
        compilation_check = self._check_compilation(generated_code)
        
        # Combine LLM evaluation with compilation check
        combined_evaluation = f"{evaluation}\n\n--- COMPILATION CHECK ---\n{compilation_check}"
        
        # Parse the combined evaluation
        evaluation_lower = combined_evaluation.lower()
        
        # Strict evaluation criteria
        if "secure & functional" in evaluation_lower and "needs improvement" not in evaluation_lower and "critical issues" not in evaluation_lower:
            # Additional verification for compilation
            if "compilation successful" in compilation_check.lower():
                is_safe = True
            else:
                is_safe = False
                combined_evaluation += "\n\nFAILED: Compilation check failed"
        elif "critical issues" in evaluation_lower:
            is_safe = False
        elif "needs improvement" in evaluation_lower:
            # Check if improvements are minor and fixable
            if self._are_improvements_minor(combined_evaluation):
                is_safe = True
                combined_evaluation += "\n\nPASSED: Minor improvements needed but code is acceptable"
            else:
                is_safe = False
        else:
            # Default to unsafe if unclear
            is_safe = False
            combined_evaluation += "\n\nFAILED: Evaluation unclear - defaulting to unsafe"
            
        return is_safe, combined_evaluation

    def _check_compilation(self, code: str) -> str:
        """
        Check if the code can be compiled/executed without syntax errors.
        Supports both Python and C++ code.
        
        Args:
            code: The code to check
            
        Returns:
            String describing compilation status
        """
        # Detect language based on code content
        is_cpp = self._is_cpp_code(code)
        
        try:
            if is_cpp:
                return self._check_cpp_compilation(code)
            else:
                return self._check_python_compilation(code)
                
        except Exception as e:
            return f"❌ COMPILATION ERROR: {str(e)}"

    def _is_cpp_code(self, code: str) -> bool:
        """
        Detect if the code is C++ based on content.
        
        Args:
            code: The code to analyze
            
        Returns:
            True if C++ code, False if Python
        """
        cpp_indicators = [
            "#include",
            "std::",
            "namespace",
            "class",
            "public:",
            "private:",
            "protected:",
            "int main(",
            "void",
            "const",
            "&",
            "->",
            "::",
            "template<",
            "typename"
        ]
        
        python_indicators = [
            "import ",
            "from ",
            "def ",
            "class ",
            "if __name__",
            "print(",
            "try:",
            "except:",
            "finally:",
            "with ",
            "as ",
            "lambda ",
            "yield ",
            "async ",
            "await "
        ]
        
        code_lower = code.lower()
        
        cpp_score = sum(1 for indicator in cpp_indicators if indicator.lower() in code_lower)
        python_score = sum(1 for indicator in python_indicators if indicator.lower() in code_lower)
        
        return cpp_score > python_score

    def _check_python_compilation(self, code: str) -> str:
        """
        Check Python code compilation.
        
        Args:
            code: Python code to check
            
        Returns:
            Compilation status string
        """
        try:
            # Create a temporary file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as temp_file:
                temp_file.write(code)
                temp_file_path = temp_file.name
            
            # Try to compile the code
            result = subprocess.run(
                [sys.executable, '-m', 'py_compile', temp_file_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            # Clean up
            os.unlink(temp_file_path)
            
            if result.returncode == 0:
                return "PYTHON COMPILATION SUCCESSFUL: Code compiles without syntax errors"
            else:
                return f"PYTHON COMPILATION FAILED: {result.stderr}"
                
        except subprocess.TimeoutExpired:
            return "PYTHON COMPILATION TIMEOUT: Code took too long to compile"
        except Exception as e:
            return f"PYTHON COMPILATION ERROR: {str(e)}"

    def _check_cpp_compilation(self, code: str) -> str:
        """
        Check C++ code compilation using available compilers.
        Supports both Windows (MSVC) and Unix (g++) environments.
        
        Args:
            code: C++ code to check
            
        Returns:
            Compilation status string
        """
        import platform
        
        # Try different compilers based on platform
        compilers = []
        
        if platform.system() == "Windows":
            # Windows: try MSVC first, then MinGW
            compilers = [
                ('cl', ['cl', '/c', '/nologo']),  # MSVC
                ('g++', ['g++', '-std=c++17', '-c']),  # MinGW
                ('clang++', ['clang++', '-std=c++17', '-c'])  # Clang
            ]
        else:
            # Unix/Linux: try g++ first, then clang++
            compilers = [
                ('g++', ['g++', '-std=c++17', '-c']),
                ('clang++', ['clang++', '-std=c++17', '-c'])
            ]
        
        for compiler_name, compiler_args in compilers:
            try:
                # Check if compiler is available
                if compiler_name == 'cl':
                    # MSVC uses different version check
                    try:
                        # Try to find MSVC in common locations
                        import subprocess
                        result = subprocess.run(['where', 'cl'], capture_output=True, text=True, timeout=5)
                        if result.returncode != 0:
                            continue
                    except:
                        continue
                else:
                    # g++ and clang++ use --version
                    try:
                        version_check = subprocess.run([compiler_args[0], '--version'], 
                                                    capture_output=True, text=True, timeout=5)
                        if version_check.returncode != 0:
                            continue
                    except:
                        continue
                
                # Create a temporary file
                suffix = '.cpp'
                if platform.system() == "Windows":
                    suffix = '.cpp'
                
                with tempfile.NamedTemporaryFile(mode='w', suffix=suffix, delete=False) as temp_file:
                    temp_file.write(code)
                    temp_file_path = temp_file.name
                
                # Prepare output file path
                if platform.system() == "Windows":
                    output_path = temp_file_path.replace('.cpp', '.obj')
                else:
                    output_path = '/dev/null'
                
                # Try to compile the code
                compile_args = compiler_args + [temp_file_path, '-o', output_path]
                result = subprocess.run(compile_args, capture_output=True, text=True, timeout=15)
                
                # Clean up
                os.unlink(temp_file_path)
                if os.path.exists(output_path):
                    os.unlink(output_path)
                
                if result.returncode == 0:
                    return f"C++ COMPILATION SUCCESSFUL: Code compiles with {compiler_name}"
                else:
                    return f"C++ COMPILATION FAILED ({compiler_name}): {result.stderr}"
                    
            except subprocess.TimeoutExpired:
                continue
            except FileNotFoundError:
                continue
            except Exception as e:
                continue
        
        return "C++ COMPILATION SKIPPED: No suitable compiler found (g++, clang++, or MSVC)"

    def _are_improvements_minor(self, evaluation: str) -> bool:
        """
        Check if the improvements needed are minor and acceptable.
        
        Args:
            evaluation: The evaluation text
            
        Returns:
            True if improvements are minor, False otherwise
        """
        minor_issues = [
            "comment",
            "variable name",
            "formatting",
            "style",
            "minor",
            "cosmetic"
        ]
        
        critical_issues = [
            "security vulnerability",
            "input validation",
            "error handling",
            "compilation error",
            "syntax error",
            "undefined",
            "import error"
        ]
        
        evaluation_lower = evaluation.lower()
        
        # Check for critical issues
        for issue in critical_issues:
            if issue in evaluation_lower:
                return False
        
        # Check if only minor issues exist
        minor_count = sum(1 for issue in minor_issues if issue in evaluation_lower)
        return minor_count > 0
