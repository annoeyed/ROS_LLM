"""This script runs the Phase 1 workflow for generating secure ROS code."""

import datetime
import json
import os
import sys
import logging
import io
from typing import Any, Dict, Optional

# Add the project root to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agents import CoderAgent, JudgeAgent, PlannerAgent, SecurityGuideAgent
from utils.ai_client import AIClientFactory
from utils.config_loader import ConfigLoader


class ConsoleCapture:
    """Captures console output and saves it to a file."""
    
    def __init__(self, log_file_path: str):
        self.log_file_path = log_file_path
        self.console_output = []
        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr
        
    def __enter__(self):
        # Create custom stdout/stderr that captures output
        self.captured_stdout = io.StringIO()
        self.captured_stderr = io.StringIO()
        
        # Create tee-like behavior - output to both console and capture
        class TeeStream:
            def __init__(self, original, captured, output_list):
                self.original = original
                self.captured = captured
                self.output_list = output_list
                
            def write(self, text):
                self.original.write(text)  # Write to console
                self.captured.write(text)  # Capture for later
                self.output_list.append(text)  # Store in list
                return len(text)
                
            def flush(self):
                self.original.flush()
                self.captured.flush()
        
        # Replace stdout and stderr
        sys.stdout = TeeStream(self.original_stdout, self.captured_stdout, self.console_output)
        sys.stderr = TeeStream(self.original_stderr, self.captured_stderr, self.console_output)
        
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        # Restore original stdout/stderr
        sys.stdout = self.original_stdout
        sys.stderr = self.original_stderr
        
        # Save captured output to file
        try:
            os.makedirs(os.path.dirname(self.log_file_path), exist_ok=True)
            with open(self.log_file_path, 'w', encoding='utf-8') as f:
                f.write(''.join(self.console_output))
            print(f"Console output saved to: {self.log_file_path}")
        except Exception as e:
            print(f"Failed to save console output: {e}")
    
    def get_captured_output(self) -> str:
        """Get the captured console output as a string."""
        return ''.join(self.console_output)


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
      raise ValueError("AI_CLIENT_TYPE environment variable is not set. Please set it to 'openai' or 'anthropic'.")
    
    if client_type == 'openai':
      model_name = ai_config.get('openai_model')
      if not ai_config.get('openai_api_key'):
        raise ValueError("OPENAI_API_KEY environment variable is not set.")
    elif client_type == 'anthropic':
      model_name = ai_config.get('anthropic_model')
      if not ai_config.get('anthropic_api_key'):
        raise ValueError("ANTHROPIC_API_KEY environment variable is not set.")
    else:
      raise ValueError(f"Unsupported client type: {client_type}. Please use 'openai' or 'anthropic'.")
    
    self.llm_client = AIClientFactory.create_client(
        client_name=client_type,
        config=ai_config,
        model_name=model_name)

    if not self.llm_client:
      raise ValueError(
          f"Failed to create AI client. Please check your {client_type} configuration.")

    # Initialize agents with the created client
    self.planner = PlannerAgent(self.llm_client)
    self.security_guide = SecurityGuideAgent(self.llm_client)
    self.coder = CoderAgent(self.llm_client)
    self.judge = JudgeAgent(self.llm_client)

  def run(self, instruction: str, language: str = "python") -> Dict[str, Any]:
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
      generated_code = self.coder.generate_code(plan, security_guidelines, language)
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
        print(f"Error occurred in Judge Agent: {e}")
        judge_duration = (datetime.datetime.now() - judge_start).total_seconds()
        is_safe = False
        feedback = f"Judge Agent execution error: {str(e)}"
      
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
          "final_feedback": feedback if feedback else "Code generation exceeded maximum retry attempts.",
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


def save_results(result: Dict[str, Any], timestamp: str = None, run_dir: str = None):
  """Saves the workflow results to a JSON file and the generated code to a Python file.

    Args:
        result: A dictionary containing the workflow results.
        timestamp: Timestamp for file naming (if None, generates new one).
        run_dir: Directory to save results (if None, creates new one).
  """
  if timestamp is None:
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
  
  if run_dir is None:
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

  # Save the generated code to appropriate file (language-specific extension)
  generated_code = result.get("generated_code", "")
  language = result.get("language", "python")  # Default to python if not specified
  
  if generated_code and not generated_code.startswith("# Code generation failed"):
    # Determine file extension based on language
    if language.lower() in ["cpp", "c++"]:
      code_extension = "cpp"
      header_comment = f"""/*
 * Generated ROS 2 C++ Node - Secure Code
 * Generated by Phase 1 Multi-Agent Workflow
 * 
 * Timestamp: {timestamp}
 * Instruction: {result.get("instruction", "N/A")}
 * Verification Status: {result.get("verification_status", "N/A")}
 * Judge Feedback: {result.get("final_feedback", "N/A")}
 */

"""
      # Generate and save CMakeLists.txt and package.xml for C++
      node_name = f"generated_ros_node_{timestamp}"
      _save_cpp_build_files(run_dir, node_name, timestamp, result)
      
    elif language.lower() == "c":
      code_extension = "c"
      header_comment = f"""/*
 * Generated ROS 2 C Node - Secure Code
 * Generated by Phase 1 Multi-Agent Workflow
 * 
 * Timestamp: {timestamp}
 * Instruction: {result.get("instruction", "N/A")}
 * Verification Status: {result.get("verification_status", "N/A")}
 * Judge Feedback: {result.get("final_feedback", "N/A")}
 */

"""
      # Generate and save CMakeLists.txt and package.xml for C
      node_name = f"generated_ros_node_{timestamp}"
      _save_c_build_files(run_dir, node_name, timestamp, result)
      
    else:
      code_extension = "py"
      header_comment = f"""#!/usr/bin/env python3
# -*- coding: utf-8 -*-
\"\"\"
Generated ROS 2 Python Node - Secure Code
Generated by Phase 1 Multi-Agent Workflow

Timestamp: {timestamp}
Instruction: {result.get("instruction", "N/A")}
Verification Status: {result.get("verification_status", "N/A")}
Judge Feedback: {result.get("final_feedback", "N/A")}
\"\"\"

"""
    
    code_filename = os.path.join(run_dir, f"generated_ros_node_{timestamp}.{code_extension}")
    with open(code_filename, "w", encoding="utf-8") as f:
      f.write(header_comment + generated_code)
    print(f"Generated code saved to {code_filename}")
    
    # Try to compile C++ or C code if applicable
    if language.lower() in ["cpp", "c++"]:
      _validate_cpp_compilation(run_dir, f"generated_ros_node_{timestamp}")
    elif language.lower() == "c":
      _validate_c_compilation(run_dir, f"generated_ros_node_{timestamp}")
      
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


def _save_c_build_files(run_dir: str, node_name: str, timestamp: str, result: Dict[str, Any]):
  """Generate and save CMakeLists.txt and package.xml for C projects"""
  
  # Generate CMakeLists.txt for C
  cmake_content = f"""cmake_minimum_required(VERSION 3.8)
project({node_name})

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c11)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(example_interfaces REQUIRED)

# Add executable
add_executable({node_name} {node_name}.c)

# Include directories
target_include_directories({node_name} PUBLIC
  $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
  $<INSTALL_INTERFACE:include>)

# Specify dependencies
ament_target_dependencies({node_name}
  rcl
  std_msgs
  rcutils
  rmw
  example_interfaces)

# Install executable
install(TARGETS {node_name}
  DESTINATION lib/${{PROJECT_NAME}})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
"""
  
  cmake_filename = os.path.join(run_dir, "CMakeLists.txt")
  with open(cmake_filename, "w", encoding="utf-8") as f:
    f.write(cmake_content)
  print(f"CMakeLists.txt saved to {cmake_filename}")
  
  # Generate package.xml for C
  package_content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{node_name}</name>
  <version>1.0.0</version>
  <description>Generated ROS 2 C secure node - {timestamp}</description>
  <maintainer email="developer@example.com">ROS LLM Generator</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rcl</depend>
  <depend>std_msgs</depend>
  <depend>rcutils</depend>
  <depend>rmw</depend>
  <depend>example_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
  
  package_filename = os.path.join(run_dir, "package.xml")
  with open(package_filename, "w", encoding="utf-8") as f:
    f.write(package_content)
  print(f"package.xml saved to {package_filename}")


def _save_cpp_build_files(run_dir: str, node_name: str, timestamp: str, result: Dict[str, Any]):
  """Generate and save CMakeLists.txt and package.xml for C++ projects"""
  
  # Generate CMakeLists.txt
  cmake_content = f"""cmake_minimum_required(VERSION 3.8)
project({node_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Add executable
add_executable({node_name} {node_name}.cpp)

# Include directories
target_include_directories({node_name} PUBLIC
  $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
  $<INSTALL_INTERFACE:include>)

# Specify dependencies
ament_target_dependencies({node_name}
  rclcpp
  std_msgs
  rclcpp_action
  example_interfaces
  std_srvs)

# Install executable
install(TARGETS {node_name}
  DESTINATION lib/${{PROJECT_NAME}})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
"""
  
  # Generate package.xml
  package_content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{node_name}</name>
  <version>1.0.0</version>
  <description>Generated ROS 2 C++ secure node - {timestamp}</description>
  <maintainer email="developer@example.com">ROS LLM Generator</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>rclcpp_action</depend>
  <depend>example_interfaces</depend>
  <depend>std_srvs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
  
  # Save files
  cmake_filename = os.path.join(run_dir, "CMakeLists.txt")
  with open(cmake_filename, "w", encoding="utf-8") as f:
    f.write(cmake_content)
  print(f"CMakeLists.txt saved to {cmake_filename}")
  
  package_filename = os.path.join(run_dir, "package.xml")
  with open(package_filename, "w", encoding="utf-8") as f:
    f.write(package_content)
  print(f"package.xml saved to {package_filename}")


def _validate_cpp_compilation(run_dir: str, node_name: str):
  """Validate C++ code compilation"""
  import subprocess
  
  try:
    print(f"\\nPhase 1 code generation review completed: {node_name}")
    
    # Check if ROS 2 is available (Windows compatible)
    try:
      # Try Windows first
      result = subprocess.run(["where", "colcon"], capture_output=True, text=True, shell=True)
      if result.returncode != 0:
        # Try Linux/Mac
        result = subprocess.run(["which", "colcon"], capture_output=True, text=True)
        if result.returncode != 0:
          print("ROS 2 colcon not found - skipping compilation validation")
          print("Phase 1 complete: Code generation and AI review finished")
          print("Compilation and execution validation will be performed in Phase 2: Evaluation")
          return
    except FileNotFoundError:
      print("ROS 2 colcon not found - skipping compilation validation")
      print("Phase 1 complete: Code generation and AI review finished")
      print("Compilation and execution validation will be performed in Phase 2: Evaluation")
      return
    
    # Try to build the package
    print("Attempting to build C++ package...")
    build_result = subprocess.run(
      ["colcon", "build", "--packages-select", node_name],
      cwd=run_dir,
      capture_output=True,
      text=True,
      timeout=60
    )
    
    if build_result.returncode == 0:
      print("C++ code compilation successful!")
      print("Build output:")
      print(build_result.stdout)
    else:
      print("C++ code compilation failed:")
      print("Error output:")
      print(build_result.stderr)
      
      # Save compilation errors to file
      error_filename = os.path.join(run_dir, f"compilation_errors_{node_name}.txt")
      with open(error_filename, "w", encoding="utf-8") as f:
        f.write("Compilation Error Log\\n")
        f.write("="*50 + "\\n\\n")
        f.write("STDOUT:\\n")
        f.write(build_result.stdout)
        f.write("\\n\\nSTDERR:\\n")
        f.write(build_result.stderr)
      print(f"Compilation errors saved to {error_filename}")
      
  except subprocess.TimeoutExpired:
    print("Compilation timeout - build took too long")
  except FileNotFoundError:
    print("Build tools not found - skipping compilation validation")
  except Exception as e:
    print(f"Compilation validation error: {e}")


def _validate_c_compilation(run_dir: str, node_name: str):
  """Validate C code compilation"""
  import subprocess
  
  try:
    print(f"\\nValidating C compilation for {node_name}...")
    
    # Check if ROS 2 is available (Windows compatible)
    try:
      # Try Windows first
      result = subprocess.run(["where", "colcon"], capture_output=True, text=True, shell=True)
      if result.returncode != 0:
        # Try Linux/Mac
        result = subprocess.run(["which", "colcon"], capture_output=True, text=True)
        if result.returncode != 0:
          print("ROS 2 colcon not found - skipping compilation validation")
          print("Phase 1 complete: Code generation and AI review finished")
          print("Compilation and execution validation will be performed in Phase 2: Evaluation")
          return
    except FileNotFoundError:
      print("ROS 2 colcon not found - skipping compilation validation")
      print("Phase 1 complete: Code generation and AI review finished")
      print("Compilation and execution validation will be performed in Phase 2: Evaluation")
      return
    
    # Try to build the package
    print("Attempting to build C package...")
    build_result = subprocess.run(
      ["colcon", "build", "--packages-select", node_name],
      cwd=run_dir,
      capture_output=True,
      text=True,
      timeout=60
    )
    
    if build_result.returncode == 0:
      print("C code compilation successful!")
      print("Build output:")
      print(build_result.stdout)
    else:
      print("C code compilation failed:")
      print("Error output:")
      print(build_result.stderr)
      
      # Save compilation errors to file
      error_filename = os.path.join(run_dir, f"compilation_errors_{node_name}.txt")
      with open(error_filename, "w", encoding="utf-8") as f:
        f.write("Compilation Error Log\\n")
        f.write("="*50 + "\\n\\n")
        f.write("STDOUT:\\n")
        f.write(build_result.stdout)
        f.write("\\n\\nSTDERR:\\n")
        f.write(build_result.stderr)
      print(f"Compilation errors saved to {error_filename}")
      
  except subprocess.TimeoutExpired:
    print("Compilation timeout - build took too long")
  except FileNotFoundError:
    print("Build tools not found - skipping compilation validation")
  except Exception as e:
    print(f"Compilation validation error: {e}")


if __name__ == "__main__":
  # Example instructions with language specification
  # pylint: disable=line-too-long
  user_instruction_python = "Create a ROS 2 node in Python that subscribes to a 'std_msgs/String' topic named 'chatter' and prints the received messages to the console. The node should be named 'listener_node'."
  user_instruction_cpp = "Create a ROS 2 node in C++ that subscribes to a 'std_msgs/String' topic named 'chatter' and prints the received messages to the console. The node should be named 'listener_node'."
  user_instruction_c = "Create a ROS 2 node in C that subscribes to a 'std_msgs/String' topic named 'chatter' and prints the received messages to the console. The node should be named 'listener_node'."
  
  # Choose language (python, cpp, or c)
  language = "cpp"  # Change this to "cpp", "c", or "python" for code generation
  if language == "python":
    user_instruction = user_instruction_python
  elif language == "c":
    user_instruction = user_instruction_c
  else:
    user_instruction = user_instruction_cpp

  # Create timestamp for this run
  timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
  
  # Create results directory for this run
  results_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "results", f"phase1_run_{timestamp}")
  os.makedirs(results_dir, exist_ok=True)
  
  # Set up console log file path
  console_log_path = os.path.join(results_dir, f"console_output_{timestamp}.log")
  
  # Initialize and run the workflow with console capture
  workflow_result = None
  
  with ConsoleCapture(console_log_path):
    workflow = Phase1_GenerationWorkflow(max_retries=5)
    
    try:
      workflow_result = workflow.run(user_instruction, language)
      # Add language info to result for proper file saving
      workflow_result["language"] = language
      save_results(workflow_result, timestamp, results_dir)
      print(f"\nWorkflow completed successfully!")
    except (RuntimeError, ValueError) as e:
      print(f"\nError occurred during workflow execution: {e}")
      
      # Save partial results if available
      if hasattr(workflow, 'partial_result') and workflow.partial_result:
        print("Saving partial results...")
        workflow.partial_result["language"] = language
        save_results(workflow.partial_result, timestamp, results_dir)
      else:
        # Generate default failure result
        failure_result = {
          "instruction": user_instruction,
          "language": language,
          "plan": "Workflow execution failed",
          "security_guidelines": "Not generated",
          "generated_code": f"# Code generation failed\n# Error: {str(e)}",
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
        print("Saving failure results...")
        save_results(failure_result, timestamp, results_dir)

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

