"""This script runs the Phase 1 workflow for generating secure ROS code."""

import datetime
import json
import os
import sys
from typing import Any, Dict, Optional

# Add the project root to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents import CoderAgent, JudgeAgent, PlannerAgent, SecurityGuideAgent
from rag_utils.ai_client import AIClientFactory
from rag_utils.config_loader import ConfigLoader


class Phase1_GenerationWorkflow:
  """Workflow for Phase 1: Generation of secure ROS code."""

  def __init__(self, max_retries: int = 3):
    """Initializes the Phase1_GenerationWorkflow.

    Args:
      max_retries: The maximum number of retries for code generation.
    """
    self.max_retries = max_retries
    self.partial_result = None

    # Load AI configuration
    config_loader = ConfigLoader()
    ai_config = config_loader.get_ai_config()
    
    # Validate configuration
    if not config_loader.validate_config(ai_config):
      raise ValueError("Invalid AI configuration. Please check your environment variables.")
    
    # Print configuration summary
    config_loader.print_config_summary(ai_config)

    # Create an AI client using the factory based on the config
    client_type = ai_config.get('client_type')
    if not client_type:
      raise ValueError("AI_CLIENT_TYPE 환경변수가 설정되지 않았습니다. 'openai' 또는 'anthropic'으로 설정하세요.")
    
    if client_type == 'openai':
      model_name = ai_config.get('openai_model')
      if not ai_config.get('openai_api_key'):
        raise ValueError("OPENAI_API_KEY 환경변수가 설정되지 않았습니다.")
    elif client_type == 'anthropic':
      model_name = ai_config.get('anthropic_model')
      if not ai_config.get('anthropic_api_key'):
        raise ValueError("ANTHROPIC_API_KEY 환경변수가 설정되지 않았습니다.")
    else:
      raise ValueError(f"지원하지 않는 클라이언트 타입입니다: {client_type}. 'openai' 또는 'anthropic'을 사용하세요.")
    
    self.llm_client = AIClientFactory.create_client(
        client_name=client_type,
        config=ai_config,
        model_name=model_name)

    if not self.llm_client:
      raise ValueError(
          f"AI 클라이언트 생성에 실패했습니다. {client_type} 설정을 확인하세요.")

    # Initialize agents with the created client
    self.planner = PlannerAgent(self.llm_client)
    self.security_guide = SecurityGuideAgent(self.llm_client)
    self.coder = CoderAgent(self.llm_client)
    self.judge = JudgeAgent(self.llm_client)

  def run(self, instruction: str) -> Dict[str, Any]:
    """Runs the Phase 1 workflow.

    Args:
      instruction: The user's instruction for code generation.

    Returns:
      A dictionary containing the results of the workflow.
    """
    print("Starting Phase 1: Generation Workflow...")
    
    # Initialize workflow tracking
    workflow_log = {
        "timestamp": datetime.datetime.now().isoformat(),
        "instruction": instruction,
        "workflow_steps": [],
        "agent_interactions": [],
        "attempts": []
    }

    # 1. Planner Agent
    print("\n=== Planner Agent Start ===")
    print("Step 1: Planner Agent is creating a plan...")
    step_start = datetime.datetime.now()
    plan = self.planner.generate_plan(instruction)
    step_duration = (datetime.datetime.now() - step_start).total_seconds()
    
    workflow_log["workflow_steps"].append({
        "step": 1,
        "agent": "PlannerAgent",
        "action": "generate_plan",
        "duration_seconds": step_duration,
        "status": "success",
        "output_preview": plan[:200] + "..." if len(plan) > 200 else plan
    })
    workflow_log["agent_interactions"].append({
        "agent": "PlannerAgent",
        "input": instruction,
        "output": plan,
        "timestamp": step_start.isoformat()
    })
    
    print("Plan created successfully.")
    print(f"Plan: {plan}")
    print("=== Planner Agent Finish ===")

    # 2. Security Guide Agent
    print("\n=== Security Guide Agent Start ===")
    print("Step 2: Security Guide Agent is generating security guidelines...")
    step_start = datetime.datetime.now()
    security_guidelines = self.security_guide.generate_guidelines(plan)
    step_duration = (datetime.datetime.now() - step_start).total_seconds()
    
    workflow_log["workflow_steps"].append({
        "step": 2,
        "agent": "SecurityGuideAgent",
        "action": "generate_guidelines",
        "duration_seconds": step_duration,
        "status": "success",
        "output_preview": security_guidelines[:200] + "..." if len(security_guidelines) > 200 else security_guidelines
    })
    workflow_log["agent_interactions"].append({
        "agent": "SecurityGuideAgent",
        "input": plan,
        "output": security_guidelines,
        "timestamp": step_start.isoformat()
    })
    
    print("Security guidelines generated successfully.")
    print(f"Security Guidelines: {security_guidelines}")
    print("=== Security Guide Agent Finish ===")

    # 3. Coder Agent with retry loop
    print("\n=== Coder Agent Start ===")
    print("Step 3: Coder Agent is generating the ROS code...")
    generated_code = ""
    is_safe = False
    feedback = ""
    
    for attempt in range(self.max_retries):
      attempt_start = datetime.datetime.now()
      print(f"Attempt {attempt + 1}/{self.max_retries} to generate code.")
      
      # Coder Agent
      coder_start = datetime.datetime.now()
      generated_code = self.coder.generate_code(plan, security_guidelines)
      coder_duration = (datetime.datetime.now() - coder_start).total_seconds()
      
      # Display generated code
      print("\n" + "="*60)
      print("GENERATED ROS CODE:")
      print("="*60)
      print(generated_code)
      print("="*60)

      # 4. Judge Agent
      print("\n=== Judge Agent Start ===")
      print("Step 4: Judge Agent is verifying the code...")
      judge_start = datetime.datetime.now()
      
      try:
        is_safe, feedback = self.judge.verify_code(generated_code, security_guidelines)
        judge_duration = (datetime.datetime.now() - judge_start).total_seconds()
      except Exception as e:
        print(f"Judge Agent에서 오류 발생: {e}")
        judge_duration = (datetime.datetime.now() - judge_start).total_seconds()
        is_safe = False
        feedback = f"Judge Agent 실행 오류: {str(e)}"
      
      attempt_duration = (datetime.datetime.now() - attempt_start).total_seconds()
      
      # Log this attempt
      attempt_log = {
        "attempt_number": attempt + 1,
        "coder_duration_seconds": coder_duration,
        "judge_duration_seconds": judge_duration,
        "total_attempt_duration_seconds": attempt_duration,
        "generated_code": generated_code,
        "is_safe": is_safe,
        "judge_feedback": feedback,
        "timestamp": attempt_start.isoformat()
      }
      workflow_log["attempts"].append(attempt_log)
      
      # Log agent interactions
      workflow_log["agent_interactions"].extend([
        {
          "agent": "CoderAgent",
          "input": {"plan": plan, "security_guidelines": security_guidelines},
          "output": generated_code,
          "timestamp": coder_start.isoformat()
        },
        {
          "agent": "JudgeAgent", 
          "input": {"code": generated_code, "guidelines": security_guidelines},
          "output": {"is_safe": is_safe, "feedback": feedback},
          "timestamp": judge_start.isoformat()
        }
      ])

      if is_safe:
        print("Code verification successful.")
        print("=== Judge Agent Finish ===")
        break
      else:
        print("Code verification failed. Providing feedback to the Coder Agent.")
        print(f"Feedback: {feedback}")
        # Provide feedback to the coder for the next attempt
        self.coder.add_feedback_to_history(feedback)
        print("=== Judge Agent Finish ===")
    else:
      print("Failed to generate secure code after maximum retries.")
      workflow_log["final_status"] = "failed"
      workflow_log["failure_reason"] = "Maximum retries exceeded"
      
      # Store partial result for saving even if failed
      result = {
          "instruction": instruction,
          "plan": plan,
          "security_guidelines": security_guidelines,
          "generated_code": generated_code,
          "verification_status": "Failed - Maximum retries exceeded",
          "final_feedback": feedback if feedback else "코드 생성이 최대 시도 횟수를 초과했습니다.",
          "workflow_log": workflow_log
      }
      self.partial_result = result
      
      # Handle failure case if needed, e.g., by raising an exception or returning a specific status
      raise RuntimeError("Code generation failed after maximum retries.")

    print("\n=== Coder Agent Finish ===")
    print("\nPhase 1: Generation Workflow completed successfully.")
    
    workflow_log["final_status"] = "success"
    workflow_log["total_attempts"] = len(workflow_log["attempts"])
    workflow_log["final_code_secure"] = is_safe

    result = {
        "instruction": instruction,
        "plan": plan,
        "security_guidelines": security_guidelines,
        "generated_code": generated_code,
        "verification_status": "Success" if is_safe else "Failed",
        "final_feedback": feedback if not is_safe else "Code is secure",
        "workflow_log": workflow_log
    }
    return result


def save_results(result: Dict[str, Any]):
  """Saves the workflow results to a JSON file and the generated code to a Python file.

    Args:
        result: A dictionary containing the workflow results.
  """
  timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
  
  # Create results directory
  output_dir = "results"
  os.makedirs(output_dir, exist_ok=True)
  
  # Create subdirectory for this run
  run_dir = os.path.join(output_dir, f"phase1_run_{timestamp}")
  os.makedirs(run_dir, exist_ok=True)

  # Save the full workflow result as JSON (including all agent interactions)
  full_result_filename = os.path.join(run_dir, f"full_workflow_result_{timestamp}.json")
  with open(full_result_filename, "w", encoding="utf-8") as f:
    json.dump(result, f, indent=2, ensure_ascii=False)
  print(f"Full workflow results saved to {full_result_filename}")

  # Save agent interactions separately for easier analysis
  if "workflow_log" in result:
    agent_log_filename = os.path.join(run_dir, f"agent_interactions_{timestamp}.json")
    with open(agent_log_filename, "w", encoding="utf-8") as f:
      json.dump(result["workflow_log"], f, indent=2, ensure_ascii=False)
    print(f"Agent interactions log saved to {agent_log_filename}")
    
    # Save summary statistics
    workflow_log = result["workflow_log"]
    summary = {
      "timestamp": timestamp,
      "instruction": result.get("instruction"),
      "final_status": workflow_log.get("final_status"),
      "total_attempts": workflow_log.get("total_attempts"),
      "final_code_secure": workflow_log.get("final_code_secure"),
      "verification_status": result.get("verification_status"),
      "workflow_duration": {
        "planner_duration": next((step["duration_seconds"] for step in workflow_log["workflow_steps"] if step["agent"] == "PlannerAgent"), 0),
        "security_guide_duration": next((step["duration_seconds"] for step in workflow_log["workflow_steps"] if step["agent"] == "SecurityGuideAgent"), 0),
        "total_attempts_duration": sum(attempt["total_attempt_duration_seconds"] for attempt in workflow_log["attempts"])
      },
      "agent_count": len(set(interaction["agent"] for interaction in workflow_log["agent_interactions"])),
      "total_interactions": len(workflow_log["agent_interactions"])
    }
    summary_filename = os.path.join(run_dir, f"workflow_summary_{timestamp}.json")
    with open(summary_filename, "w", encoding="utf-8") as f:
      json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"Workflow summary saved to {summary_filename}")

  # Save the generated code to a .py file (only if code was actually generated)
  generated_code = result.get("generated_code", "")
  if generated_code and not generated_code.startswith("# 코드 생성 실패"):
    code_filename = os.path.join(run_dir, f"generated_ros_node_{timestamp}.py")
    with open(code_filename, "w", encoding="utf-8") as f:
      # Add header comment with metadata
      header = f"""#!/usr/bin/env python3
# -*- coding: utf-8 -*-
\"\"\"
Generated ROS 2 Node - Secure Code
Generated by Phase 1 Multi-Agent Workflow

Timestamp: {timestamp}
Instruction: {result.get("instruction", "N/A")}
Verification Status: {result.get("verification_status", "N/A")}
Judge Feedback: {result.get("final_feedback", "N/A")}
\"\"\"

"""
      f.write(header + generated_code)
    print(f"Generated code saved to {code_filename}")
  else:
    print("No valid code generated - skipping code file save")

  # Save judge feedback separately for analysis
  if result.get("final_feedback") and result.get("final_feedback") != "N/A":
    feedback_filename = os.path.join(run_dir, f"judge_feedback_{timestamp}.txt")
    with open(feedback_filename, "w", encoding="utf-8") as f:
      f.write(f"Judge Agent Feedback - {timestamp}\n")
      f.write("="*50 + "\n\n")
      f.write(f"Instruction: {result.get('instruction')}\n\n")
      f.write(f"Verification Status: {result.get('verification_status')}\n\n")
      f.write("Feedback:\n")
      f.write(result.get("final_feedback"))
    print(f"Judge feedback saved to {feedback_filename}")
  
  print(f"\nAll results saved in directory: {run_dir}")
  return run_dir


if __name__ == "__main__":
  # Example instruction
  # pylint: disable=line-too-long
  user_instruction = "Create a ROS 2 node in Python that subscribes to a 'std_msgs/String' topic named 'chatter' and prints the received messages to the console. The node should be named 'listener_node'."

  # Initialize and run the workflow
  workflow = Phase1_GenerationWorkflow(max_retries=5)
  workflow_result = None
  
  try:
    workflow_result = workflow.run(user_instruction)
    save_results(workflow_result)
    print(f"\n워크플로우가 성공적으로 완료되었습니다!")
  except (RuntimeError, ValueError) as e:
    print(f"\n워크플로우 실행 중 오류가 발생했습니다: {e}")
    
    # 실패한 경우에도 부분적인 결과가 있다면 저장
    if hasattr(workflow, 'partial_result') and workflow.partial_result:
      print("부분적인 결과를 저장하는 중...")
      save_results(workflow.partial_result)
    else:
      # 기본 실패 결과 생성
      failure_result = {
        "instruction": user_instruction,
        "plan": "워크플로우 실행 실패",
        "security_guidelines": "생성되지 않음",
        "generated_code": f"# 코드 생성 실패\n# 오류: {str(e)}",
        "verification_status": "Failed",
        "final_feedback": str(e),
        "workflow_log": {
          "timestamp": datetime.datetime.now().isoformat(),
          "instruction": user_instruction,
          "final_status": "failed",
          "failure_reason": str(e),
          "workflow_steps": [],
          "agent_interactions": [],
          "attempts": []
        }
      }
      print("실패 결과를 저장하는 중...")
      save_results(failure_result)

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

