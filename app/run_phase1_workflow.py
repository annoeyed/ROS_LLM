import os
import json
from datetime import datetime
import sys

# Add the root directory of the project to the Python path.
# This allows us to import modules from other directories, such as 'agents' and 'rag_utils'.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents import PlannerAgent, SecurityGuideAgent, CoderAgent, JudgeAgent
from rag_utils.ai_client import AIClient


class Phase1_GenerationWorkflow:
    """
    Orchestrates the Phase 1 workflow for generating and evaluating secure ROS code.
    The workflow consists of four main agents: Planner, Security Guide, Coder, and Judge.
    It includes a feedback loop for code correction based on the Judge's evaluation.
    """
    def __init__(self, max_retries=3):
        """
        Initializes the workflow by setting up the AI client and all the necessary agents.
        """
        self.llm_client = AIClient()
        self.planner_agent = PlannerAgent(self.llm_client)
        self.security_guide_agent = SecurityGuideAgent(self.llm_client)
        self.coder_agent = CoderAgent(self.llm_client)
        self.judge_agent = JudgeAgent(self.llm_client)
        self.max_retries = max_retries

    def run(self, instruction: str):
        """
        Executes the entire Phase 1 workflow based on a user instruction.

        Args:
            instruction: The user's request for code generation.

        Returns:
            A dictionary containing the outputs from each agent and the feedback history.
        """
        print("Starting Phase 1: Secure Code Generation Workflow...")
        print("-" * 50)

        # === 1. Planner Agent: Create a step-by-step plan ===
        print("=== Planner Agent Start ===")
        plan = self.planner_agent.run(instruction)
        print(f"Plan:\n{plan}")
        print("=== Planner Agent Finish ===\n")

        # === 2. Security Guide Agent: Generate security guidelines ===
        print("=== Security Guide Agent Start ===")
        security_guidelines = self.security_guide_agent.run(plan)
        print(f"Security Guidelines:\n{security_guidelines}")
        print("=== Security Guide Agent Finish ===\n")

        # === 3. Coder and Judge Agents with Feedback Loop ===
        feedback_history = []
        generated_code = ""
        judgment = ""
        
        for attempt in range(self.max_retries):
            print(f"--- Attempt {attempt + 1} of {self.max_retries} ---")
            
            # --- Coder Agent ---
            print("=== Coder Agent Start ===")
            if attempt == 0:
                # First attempt: Use the original plan and guidelines
                coder_input = f"## Plan:\n{plan}\n\n## Security Guidelines:\n{security_guidelines}"
            else:
                # Subsequent attempts: Provide feedback from the judge
                print("Applying feedback to regenerate code...")
                coder_input = f"""
## Original Plan:
{plan}

## Security Guidelines:
{security_guidelines}

## Previous Code Attempt (Attempt #{attempt}):
```python
{generated_code}
```

## Feedback from Security Auditor:
{judgment}

## Your Task:
Please regenerate the Python code for the ROS node, carefully addressing all the feedback provided by the auditor to fix the security issues. Ensure the new code is secure and correct.
"""
            generated_code = self.coder_agent.run(coder_input)
            print(f"Generated Code (Attempt {attempt + 1}):\n{generated_code}")
            print("=== Coder Agent Finish ===\n")

            # --- Judge Agent ---
            print("=== LLM as Judge Agent Start ===")
            judge_input = f"""
## Initial Instruction:
{instruction}

## Security Guidelines:
{security_guidelines}

## Generated Code to Evaluate:
```python
{generated_code}
```
"""
            judgment = self.judge_agent.run(judge_input)
            print(f"Judgment (Attempt {attempt + 1}):\n{judgment}")
            print("=== LLM as Judge Agent Finish ===\n")

            # Store the history of this attempt
            feedback_history.append({
                "attempt": attempt + 1,
                "generated_code": generated_code,
                "judgment": judgment
            })

            # Check if the code is secure
            if "secure" in judgment.lower():
                print("Judgment is 'Secure'. Proceeding...")
                break
            else:
                print(f"Judgment is not 'Secure'. Retrying... ({self.max_retries - 1 - attempt} retries left)")
        
        print("-" * 50)
        if "secure" in judgment.lower():
            print("Workflow finished successfully!")
        else:
            print("Workflow finished, but the final code may still be insecure after maximum retries.")


        # Compile all the results into a single dictionary.
        result = {
            "instruction": instruction,
            "plan": plan,
            "security_guidelines": security_guidelines,
            "feedback_history": feedback_history,
            "final_code": generated_code
        }
        return result

    def save_result(self, result: dict):
        """
        Saves the workflow result to a JSON file in the 'results' directory.
        """
        output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results')
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = os.path.join(output_dir, f"phase1_result_{timestamp}.json")

        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=4)

        print(f"Result saved to: {file_path}")


if __name__ == "__main__":
    # Example instruction based on the 'Access Control' concept from the official ROS 2 security guidelines.
    user_instruction = """
Create a secure ROS 2 node named 'secure_parameter_node'.
This node must declare a string parameter 'robot_name' with a default value 'default_bot'.
Crucially, to prevent unauthorized runtime changes, this parameter must be initialized as 'read-only'.
The node should also have a timer that periodically prints the value of this parameter to the console to verify its state.
"""

    workflow = Phase1_GenerationWorkflow(max_retries=3)
    final_result = workflow.run(user_instruction)

    workflow.save_result(final_result)

# === To-do: Phase 2 and Feedback Loop Structure ===
#
# Phase 2: Evaluation (Oracle Verification)
#   Simulation → Oracles (Param, Safety, Mode)
#   - Simulation: Execute SITL or actual hardware simulation.
#   - Oracles: Perform parallel verification.
#     - Param Oracle: Verify safety ranges for parameters like velocity, altitude, angle, etc.
#     - Safety Oracle: Verify compliance with emergency procedures, forbidden APIs, and policies.
#     - Mode Oracle: Verify the mode sequence of the task specification.
#
# Feedback Loop Structure
#   - Param Oracle Failure → Feedback to Coder → Re-verify after parameter modification.
#   - Safety Oracle Failure → Feedback to Security Guide → Re-run after guideline modification.
#   - Mode Oracle Failure → Feedback to Planner → Re-run after plan modification.

