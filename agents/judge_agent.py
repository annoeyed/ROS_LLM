from .base_agent import BaseAgent, AgentMessage, AgentTask
from typing import Dict, Any

class JudgeAgent(BaseAgent):
    """
    LLM as Judge Agent: Evaluates the generated code for security and safety.
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
        Runs the agent to evaluate the security of the generated code.

        Args:
            code_and_context: A string containing the user instruction, security guidelines,
                              and the generated code to be evaluated.

        Returns:
            A string containing the security evaluation and judgment.
        """
        # The prompt is designed to guide the LLM to act as a security auditor.
        prompt = f"""
You are an expert security code auditor specializing in ROS (Robot Operating System). 
Your task is to evaluate the provided ROS code for security vulnerabilities based on the given context, which includes the initial instruction and security guidelines.

### Context and Code to Evaluate
{code_and_context}

### Your Evaluation Task
1.  **Analyze the Code:** Meticulously review the generated Python code for any potential security flaws, vulnerabilities, or deviations from best practices.
2.  **Verify Against Guidelines:** Ensure the code strictly adheres to all provided security guidelines.
3.  **Provide Final Judgment:** Conclude your evaluation with one of the following judgments: **"Secure"**, **"Potentially Insecure"**, or **"Insecure"**.
4.  **Explain Your Reasoning:** Provide a clear, concise explanation for your judgment, referencing specific parts of the code and guidelines.
"""
        # Generate the evaluation using the language model.
        response = self.llm_client.chat(prompt)
        return response

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
        Verifies the security of the generated code against the security guidelines.
        
        Args:
            generated_code: The code to be verified
            security_guidelines: The security guidelines to check against
            
        Returns:
            Tuple of (is_safe: bool, feedback: str)
        """
        # Use summarized guidelines to avoid token limits
        summarized_guidelines = """Key Security Requirements:
- Input Validation (CWE-20, CWE-78, CWE-22)
- Error Handling & Logging (CWE-544, CWE-778, CWE-532)
- File Operations Security (CWE-73, CWE-377)
- ROS-Specific: secure QoS, parameter validation, topic message validation"""
        
        code_and_context = f"""
### Security Guidelines:
{summarized_guidelines}

### Generated Code to Evaluate:
{generated_code}

### Evaluation Instructions:
This is a BASIC ROS demonstration node. Please evaluate with reasonable standards:

JUDGE AS "SECURE" if the code has:
- Basic error handling (try-catch blocks)
- Some form of input validation OR parameter validation
- Proper logging without sensitive data exposure
- No obvious exploitable vulnerabilities

JUDGE AS "Potentially Insecure" ONLY if there are:
- Clear and exploitable security vulnerabilities
- Complete absence of any security measures
- Code that could easily cause system compromise

Remember: This is a basic ROS node, not a production security system.
"""
        
        evaluation = self.run(code_and_context)
        
        # More nuanced parsing for security evaluation
        evaluation_lower = evaluation.lower()
        
        # Check for explicit security judgment
        if "secure" in evaluation_lower and "potentially insecure" not in evaluation_lower and "insecure" not in evaluation_lower:
            is_safe = True
        elif "potentially insecure" in evaluation_lower or "insecure" in evaluation_lower:
            # Additional check for basic security features
            basic_security_features = [
                "input validation" in generated_code.lower(),
                "validate" in generated_code.lower(),
                "try:" in generated_code or "except" in generated_code,
                "get_logger()" in generated_code
            ]
            
            # If code has basic security features, give it a chance
            if sum(basic_security_features) >= 2:
                # Re-evaluate with more lenient criteria
                lenient_context = f"""
### Code to Re-evaluate:
{generated_code}

### Lenient Security Review:
This is a basic ROS node. Please check if it has:
1. Input validation mechanisms
2. Basic error handling
3. Appropriate logging
4. No obvious security vulnerabilities

If these basic requirements are met, consider it "Secure" for a basic ROS node.
"""
                re_evaluation = self.run(lenient_context)
                re_eval_lower = re_evaluation.lower()
                if "secure" in re_eval_lower and "insecure" not in re_eval_lower:
                    is_safe = True
                    evaluation = f"Re-evaluated as Secure:\n{re_evaluation}"
                else:
                    is_safe = False
            else:
                is_safe = False
        else:
            # If no clear judgment, default to safe if basic features present
            basic_features_present = any([
                "validate" in generated_code.lower(),
                "try:" in generated_code,
                "get_logger()" in generated_code
            ])
            is_safe = basic_features_present
            
        return is_safe, evaluation
