from .base_agent import BaseAgent

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
        super().__init__(llm_client)

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
        response = self.llm_client.generate(prompt)
        return response
