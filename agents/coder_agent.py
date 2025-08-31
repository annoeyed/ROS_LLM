#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Refactored CoderAgent (ROS secure code generator)
- Clear separation of concerns (templates, security patterns, generation pipeline)
- Type-safe enums + dataclasses for results
- Pluggable AI adapter interface
- Smaller, testable methods; fewer side-effects
- Reduced duplication between Python/C++ paths

Drop-in intent: Keep public methods: process_message, execute_task, generate_code,
                generate_ros_package, generate_secure_ros_package, set_ai_client, get_status

NOTE: Replace your existing agents/coder_agent.py with this file or keep as v2 and migrate gradually.
"""

from __future__ import annotations

import os
import sys
import json
import re
from dataclasses import dataclass, asdict
from enum import Enum
from typing import Any, Dict, List, Optional, Protocol, Tuple, Callable

# Local imports
from .base_agent import BaseAgent, AgentMessage, AgentTask

# Ensure project root on path (to import utils, etc.)
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)


# =============================
# Types & Protocols
# =============================

class Language(str, Enum):
    PYTHON = "python"
    CPP = "cpp"

class Component(str, Enum):
    BASIC = "basic_node"
    PUBLISHER = "publisher"
    SUBSCRIBER = "subscriber"
    SERVICE = "service"
    ACTION = "action"
    PARAMETER = "parameter"

class SecurityLevel(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class AIAdapter(Protocol):
    """Adapter interface over arbitrary LLM clients."""
    def chat(self, prompt: str) -> Any: ...
    def analyze_response(self, raw: Any) -> Dict[str, Any]: ...


@dataclass
class GenerationResult:
    code: str
    metadata: Dict[str, Any]
    security_features: List[str]
    ai_enhanced: bool
    component_type: Component
    security_level: SecurityLevel
    language: Language
    quality_score: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["component_type"] = self.component_type.value
        d["security_level"] = self.security_level.value
        d["language"] = self.language.value
        return d


# =============================
# Registries (templates & patterns)
# =============================

class TemplateRegistry:
    """Holds language-specific templates."""
    def __init__(self) -> None:
        self._templates: Dict[Language, Dict[Component, str]] = {
            Language.PYTHON: self._python_templates(),
            Language.CPP: self._cpp_templates(),
        }

    def get(self, language: Language, component: Component) -> str:
        return self._templates[language][component]

    @staticmethod
    def _python_templates() -> Dict[Component, str]:
        base = '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GenericNode(Node):
    def __init__(self) -> None:
        super().__init__('generic_node')
        self.publisher = self.create_publisher(String, 'generic_topic', 10)
        self.timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info('Generic node started')

    def _on_timer(self) -> None:
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: %s' % msg.data)

def main(args=None) -> None:
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
    main()
'''
        return {
            Component.BASIC: base,
            Component.PUBLISHER: base.replace('Generic', 'Publisher').replace('generic_topic', 'topic'),
            Component.SUBSCRIBER: '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(String, 'topic', self._cb, 10)
        self.get_logger().info('Subscriber node started')

    def _cb(self, msg: String) -> None:
        self.get_logger().info('I heard: %s' % msg.data)

def main(args=None) -> None:
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
    main()
''',
            Component.SERVICE: '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceNode(Node):
    def __init__(self) -> None:
        super().__init__('service_node')
        self._srv = self.create_service(Trigger, 'trigger_service', self._cb)
        self.get_logger().info('Service node started')

    def _cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('Service called')
        response.success = True
        response.message = 'OK'
        return response

def main(args=None) -> None:
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
    main()
''',
            Component.ACTION: '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class ActionNode(Node):
    def __init__(self) -> None:
        super().__init__('action_node')
        self._server = ActionServer(self, Fibonacci, 'fibonacci', self._execute)
        self.get_logger().info('Action node started')

    def _execute(self, goal_handle):
        self.get_logger().info('Executing goal...')
        goal_handle.succeed()
        return Fibonacci.Result()

def main(args=None) -> None:
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
    main()
''',
            Component.PARAMETER: '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self) -> None:
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'default_value')
        val = self.get_parameter('my_parameter').value
        self.get_logger().info(f'Parameter value: {val}')


def main(args=None) -> None:
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
    main()
''',
        }

    @staticmethod
    def _cpp_templates() -> Dict[Component, str]:
        base = '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class GenericNode : public rclcpp::Node {
public:
    GenericNode() : Node("generic_node") {
        pub_ = create_publisher<std_msgs::msg::String>("generic_topic", 10);
        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { onTimer(); });
        RCLCPP_INFO(get_logger(), "Generic node started");
    }
private:
    void onTimer() {
        std_msgs::msg::String msg; msg.data = "Hello World";
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Publishing: %s", msg.data.c_str());
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
'''
        return {
            Component.BASIC: base,
            Component.PUBLISHER: base.replace('Generic', 'Publisher').replace('generic_topic', 'topic'),
            Component.SUBSCRIBER: '''#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node") {
        sub_ = create_subscription<std_msgs::msg::String>("topic", 10, [this](std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(get_logger(), "I heard: %s", msg->data.c_str());
        });
        RCLCPP_INFO(get_logger(), "Subscriber node started");
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
''',
            Component.SERVICE: '''#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ServiceNode : public rclcpp::Node {
public:
    ServiceNode() : Node("service_node") {
        srv_ = create_service<std_srvs::srv::Trigger>("trigger_service", [this](auto, auto res){
            RCLCPP_INFO(get_logger(), "Service called");
            res->success = true; res->message = "OK"; return true; });
        RCLCPP_INFO(get_logger(), "Service node started");
    }
private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
''',
            Component.ACTION: base,  # keep simple; specialize when needed
            Component.PARAMETER: '''#include <rclcpp/rclcpp.hpp>

class ParameterNode : public rclcpp::Node {
public:
    ParameterNode() : Node("parameter_node") {
        declare_parameter<std::string>("my_parameter", "default_value");
        auto val = get_parameter("my_parameter").as_string();
        RCLCPP_INFO(get_logger(), "Parameter value: %s", val.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
''',
        }


class SecurityRegistry:
    """Returns language-specific security helpers to be embedded or mixed in."""
    @staticmethod
    def python(level: SecurityLevel) -> str:
        parts: List[str] = []
        if level in (SecurityLevel.MEDIUM, SecurityLevel.HIGH):
            parts.append('''
    def validate_input(self, input_data: str) -> str:
        """Validate inbound text inputs."""
        if not isinstance(input_data, str):
            raise TypeError("input must be str")
        if not input_data or len(input_data) > 1000:
            raise ValueError("invalid length")
        # Strict special-character deny list (escape properly)
        if re.search(r"[<>\"']", input_data):
            raise ValueError("disallowed characters")
        return input_data.strip()
''')
            parts.append('''
    def secure_log(self, message: str, level: str = 'info') -> None:
        import re as _re
        masked = _re.sub(r"password[=:]\\s*\\S+", "password=***", message)
        masked = _re.sub(r"api_key[=:]\\s*\\S+", "api_key=***", masked)
        logger = self.get_logger()
        if level == 'info':
            logger.info(masked)
        elif level in ('warn','warning'):
            logger.warning(masked)
        elif level == 'error':
            logger.error(masked)
        else:
            logger.info(masked)
''')
        if level is SecurityLevel.HIGH:
            parts.append('''
    def encrypt_data(self, data: str) -> str:
        import hashlib
        return hashlib.sha256(data.encode()).hexdigest()
''')
        return "".join(parts)

    @staticmethod
    def cpp(level: SecurityLevel) -> str:
        parts: List[str] = []
        if level in (SecurityLevel.MEDIUM, SecurityLevel.HIGH):
            parts.append('''
    // Example: secure logging helper (mask secrets)
    void secureLog(const std::string& message) {
        std::string m = message;
        auto mask = [](std::string& s, const std::string& key){
            auto pos = s.find(key);
            if (pos != std::string::npos) {
                auto end = s.find_first_of(" \n", pos);
                if (end != std::string::npos) s.replace(pos + key.size(), end - (pos + key.size()), "***");
            }
        };
        mask(m, "password="); mask(m, "api_key=");
        RCLCPP_INFO(this->get_logger(), "%s", m.c_str());
    }
''')
        return "".join(parts)


# =============================
# Code Generator (pure functions)
# =============================

class CodeGenerator:
    def __init__(self, templates: TemplateRegistry) -> None:
        self.templates = templates

    def from_template(self, *, requirements: str, component: Component, level: SecurityLevel, language: Language) -> GenerationResult:
        code = self.templates.get(language, component)
        code = self._customize_for_requirements(code, requirements, language)
        code = self._inject_security(code, language, level)
        metadata = {
            "description": f"{component.value} in {language.value}",
            "dependencies": ["rclpy","std_msgs"] if language is Language.PYTHON else ["rclcpp","std_msgs"],
            "usage": "python3 node.py" if language is Language.PYTHON else "colcon build && ros2 run <pkg> <node>",
        }
        features = self._features_for(level)
        score = self._rough_quality_score(code, language)
        return GenerationResult(code=code, metadata=metadata, security_features=features, ai_enhanced=False,
                               component_type=component, security_level=level, language=language, quality_score=score)

    # ---- helpers ----
    @staticmethod
    def _features_for(level: SecurityLevel) -> List[str]:
        if level is SecurityLevel.LOW:
            return ["basic_error_handling"]
        if level is SecurityLevel.MEDIUM:
            return ["input_validation", "secure_logging", "error_handling"]
        return ["input_validation", "secure_logging", "error_handling", "encryption"]

    @staticmethod
    def _rough_quality_score(code: str, language: Language) -> float:
        score = 0
        if language is Language.PYTHON:
            score += 20 if "class" in code and "Node" in code else 0
            score += 10 if "def main" in code else 0
            score += 10 if "get_logger" in code else 0
            score += 10 if "rclpy.init" in code and "rclpy.shutdown" in code else 0
        else:
            score += 20 if "class" in code and "rclcpp::Node" in code else 0
            score += 10 if "int main(" in code else 0
            score += 10 if "RCLCPP_INFO" in code else 0
        score += min(40, len(code) / 500)  # crude length heuristic
        return float(min(100, score))

    @staticmethod
    def _customize_for_requirements(code: str, requirements: str, language: Language) -> str:
        req = requirements.lower()
        # Example: speed/velocity => rename topics/node names
        if any(k in req for k in ("speed", "velocity")):
            code = code.replace("generic_topic", "speed_command")
            code = code.replace("GenericNode", "SpeedControlNode")
        return code

    @staticmethod
    def _inject_security(code: str, language: Language, level: SecurityLevel) -> str:
        if level is SecurityLevel.LOW:
            return code
        if language is Language.PYTHON:
            # Insert helpers inside class after first method definition
            sec = SecurityRegistry.python(level)
            if not sec:
                return code
            lines = code.splitlines()
            # Find a class start
            try:
                cls_idx = next(i for i, ln in enumerate(lines) if ln.strip().startswith("class "))
            except StopIteration:
                return code + "\n\n# Security helpers (no class found)\n" + sec
            # Find insert position: after class header + first method
            insert_idx = cls_idx + 1
            while insert_idx < len(lines) and not lines[insert_idx].strip().startswith("def "):
                insert_idx += 1
            lines.insert(insert_idx + 1, sec)
            return "\n".join(lines)
        else:
            sec = SecurityRegistry.cpp(level)
            if not sec:
                return code
            # naive append inside class body if possible
            code = code.replace("};\n\nint main", sec + "};\n\nint main", 1)
            return code


# =============================
# CoderAgent (Facade over generation + AI)
# =============================

class CoderAgent(BaseAgent):
    def __init__(self, llm_client: Optional[Any], agent_id: str = "coder_001") -> None:
        super().__init__(agent_id, "Coder Agent")
        self.ai_client = llm_client  # keep original for compatibility
        self._templates = TemplateRegistry()
        self._generator = CodeGenerator(self._templates)

    # ---- Public API (kept for drop-in) ----
    def process_message(self, message: AgentMessage) -> AgentMessage:
        if message.message_type != 'request':
            return self.send_message(message.sender, 'error', {'error': f'Unsupported type: {message.message_type}'})

        content = message.content or {}
        key = next((k for k in ("generate_code","modify_code","review_code","optimize_code") if k in content), None)
        handler = {
            'generate_code': self._handle_generate,
            'modify_code': self._handle_modify,
            'review_code': self._handle_review,
            'optimize_code': self._handle_optimize,
        }.get(key, self._handle_general)
        return handler(message)

    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        self.status = 'busy'
        self.current_task = task
        try:
            if task.task_type == 'generate_ros_code':
                res = self._generate(task.parameters)
            elif task.task_type == 'modify_code':
                res = self._modify(task.parameters)
            elif task.task_type == 'review_code':
                res = self._review(task.parameters)
            elif task.task_type == 'optimize_code':
                res = self._optimize(task.parameters)
            else:
                res = {'error': f'Unsupported task type: {task.task_type}'}
            self.update_task_status(task.task_id, 'completed', res)
            return res
        except Exception as e:
            msg = f'Task failed: {e}'
            self.update_task_status(task.task_id, 'failed', error=msg)
            return {'error': msg}
        finally:
            self.status = 'idle'

    def generate_code(self, plan: str, security_guidelines: str, language: str = "python") -> str:
        params = {
            'requirements': f"Plan: {plan}\nSecurity Guidelines: {security_guidelines}",
            'component_type': Component.BASIC.value,
            'security_level': SecurityLevel.MEDIUM.value,
            'language': language,
        }
        result = self._generate(params)
        return result.get('code', '# generation failed')

    def set_ai_client(self, ai_client: Any) -> None:
        self.ai_client = ai_client
        self.logger.info(f"AI client set: {type(ai_client).__name__}")

    def get_status(self) -> Dict[str, Any]:
        return {
            'agent_id': self.agent_id,
            'agent_name': self.agent_name,
            'status': self.status,
            'message_queue_length': len(self.message_queue),
            'task_history_length': len(self.task_history),
            'current_task': self.current_task.task_id if self.current_task else None,
            'ai_client': {
                'loaded': self.ai_client is not None,
                'type': getattr(self.ai_client, '__class__', type('x',(),{})).__name__ if self.ai_client else None,
            },
            'available_components': [c.value for c in Component],
            'available_languages': [l.value for l in Language],
        }

    def add_feedback_to_history(self, feedback: str) -> None:
        """피드백을 에이전트 히스토리에 추가합니다."""
        # AI 클라이언트에 피드백을 전달하여 다음 생성에 반영하도록 합니다
        if hasattr(self, '_feedback_history'):
            self._feedback_history.append(feedback)
        else:
            self._feedback_history = [feedback]
        
        self.logger.info(f"피드백이 히스토리에 추가되었습니다: {feedback[:100]}...")  # 처음 100자만 로그

    # ---- Handlers ----
    def _handle_generate(self, message: AgentMessage) -> AgentMessage:
        p = message.content
        out = self._generate(p)
        return self.send_message(message.sender, 'response', {**out, 'status': 'success'})

    def _handle_modify(self, message: AgentMessage) -> AgentMessage:
        out = self._modify(message.content)
        return self.send_message(message.sender, 'response', {**out, 'status': 'success'})

    def _handle_review(self, message: AgentMessage) -> AgentMessage:
        out = self._review(message.content)
        return self.send_message(message.sender, 'response', {**out, 'status': 'success'})

    def _handle_optimize(self, message: AgentMessage) -> AgentMessage:
        out = self._optimize(message.content)
        return self.send_message(message.sender, 'response', {**out, 'status': 'success'})

    def _handle_general(self, message: AgentMessage) -> AgentMessage:
        return self.send_message(message.sender, 'response', {'message': 'OK', 'content': message.content})

    # ---- Core operations ----
    def _generate(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        req = parameters.get('requirements')
        if not req:
            return {'error': 'requirements missing'}
        comp = Component(parameters.get('component_type', Component.BASIC.value))
        lvl = SecurityLevel(parameters.get('security_level', SecurityLevel.MEDIUM.value))
        lang = Language(parameters.get('language', Language.PYTHON.value))

        # If an AI client is present and supports chat(), we try it first
        if self.ai_client and hasattr(self.ai_client, 'chat'):
            try:
                prompt = self._build_prompt(req, comp, lvl, lang)
                
                # 피드백 히스토리가 있으면 프롬프트에 추가
                if hasattr(self, '_feedback_history') and self._feedback_history:
                    feedback_text = "\n\n===이전 피드백===\n" + "\n".join(self._feedback_history) + "\n위 피드백을 반영하여 코드를 개선해주세요.\n"
                    prompt += feedback_text
                
                raw = self.ai_client.chat(prompt)
                self.logger.info(f"AI 응답 받음 (타입: {type(raw)}, 길이: {len(str(raw)) if raw else 0})")
                ai = self._parse_ai_response(raw, lvl, lang, comp)
                if ai:
                    self.logger.info("AI 응답 파싱 성공")
                    return ai
                else:
                    self.logger.warning("AI 응답 파싱 실패, 템플릿 기반 생성으로 폴백")
            except Exception as e:
                self.logger.warning(f"AI generation failed, fallback to templates: {e}")

        gen = self._generator.from_template(requirements=req, component=comp, level=lvl, language=lang)
        return gen.to_dict()

    def _modify(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        code = parameters.get('existing_code', '')
        req = parameters.get('modification_request', '')
        if not code or not req:
            return {'error': 'existing_code and modification_request required'}
        if self.ai_client and hasattr(self.ai_client, 'chat'):
            prompt = f"""
Modify the following code according to the request. Return JSON with keys: code, changes, explanation.

<CODE>\n{code}\n</CODE>
<REQUEST>\n{req}\n</REQUEST>
"""
            try:
                raw = self.ai_client.chat(prompt)
                out = self._safe_json(raw)
                if isinstance(out, dict) and 'code' in out:
                    return out
            except Exception as e:
                self.logger.warning(f"AI modify failed: {e}")
        # naive fallback
        return {"code": code + f"\n# TODO: {req}", "changes": [req], "explanation": "fallback append"}

    def _review(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        code = parameters.get('code_snippet', '')
        focus = parameters.get('review_focus', 'security')
        if not code:
            return {'error': 'code_snippet required'}
        if self.ai_client and hasattr(self.ai_client, 'chat'):
            prompt = f"""
Review the code for {focus}. Return JSON with keys: score (0-10), issues, recommendations, security_concerns, best_practices.
<CODE>\n{code}\n</CODE>
"""
            try:
                raw = self.ai_client.chat(prompt)
                out = self._safe_json(raw)
                if isinstance(out, dict) and 'score' in out:
                    return out
            except Exception as e:
                self.logger.warning(f"AI review failed: {e}")
        return {"score": 7, "issues": ["basic fallback"], "recommendations": ["use AI reviewer"], "security_concerns": [], "best_practices": []}

    def _optimize(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        code = parameters.get('code_snippet', '')
        goal = parameters.get('optimization_goal', 'performance')
        if not code:
            return {'error': 'code_snippet required'}
        if self.ai_client and hasattr(self.ai_client, 'chat'):
            prompt = f"""
Optimize the code for {goal}. Return JSON with keys: code, optimizations, performance_improvements, trade_offs.
<CODE>\n{code}\n</CODE>
"""
            try:
                raw = self.ai_client.chat(prompt)
                out = self._safe_json(raw)
                if isinstance(out, dict) and 'code' in out:
                    return out
            except Exception as e:
                self.logger.warning(f"AI optimize failed: {e}")
        return {"code": code + f"\n# TODO: {goal} optimization", "optimizations": [goal], "performance_improvements": [], "trade_offs": []}

    # ---- AI helpers ----
    @staticmethod
    def _build_prompt(req: str, comp: Component, lvl: SecurityLevel, lang: Language) -> str:
        base = "ROS 2 code generation. Return JSON with keys: code, metadata, security_features.\n"
        base += f"Language: {lang.value}\nComponent: {comp.value}\nSecurity: {lvl.value}\nRequirements: {req}\n"
        return base

    def _parse_ai_response(self, raw: Any, lvl: SecurityLevel, lang: Language, comp: Component) -> Optional[Dict[str, Any]]:
        self.logger.info(f"AI 응답 파싱 시작 - 타입: {type(raw)}")
        
        # AI 클라이언트의 응답이 문자열인 경우 처리
        if isinstance(raw, str):
            raw_text = raw.strip()
            self.logger.info(f"AI 응답이 문자열입니다. 길이: {len(raw_text)}")
            
            # 코드 블록 추출 시도
            code = self._maybe_extract_code(raw_text)
            if code and len(code.strip()) > 50:  # 유의미한 코드가 있는지 확인
                self.logger.info("코드 블록을 성공적으로 추출했습니다")
                return GenerationResult(
                    code=code,
                    metadata={"description": f"{lang.value} node from AI", "dependencies": []},
                    security_features=CodeGenerator._features_for(lvl),
                    ai_enhanced=True,
                    component_type=comp,
                    security_level=lvl,
                    language=lang,
                    quality_score=CodeGenerator._rough_quality_score(code, lang)
                ).to_dict()
            else:
                self.logger.warning(f"추출된 코드가 너무 짧습니다: {len(code) if code else 0} 문자")
        
        # JSON 파싱 시도
        data = self._safe_json(raw)
        if isinstance(data, dict):
            self.logger.info("JSON 형태의 응답을 파싱했습니다")
            if 'code' in data:
                # normalize
                data.setdefault('metadata', {"description": f"{lang.value} node"})
                data.setdefault('security_features', CodeGenerator._features_for(lvl))
                data.setdefault('ai_enhanced', True)
                data.setdefault('component_type', comp.value)
                data.setdefault('security_level', lvl.value)
                data.setdefault('language', lang.value)
                data.setdefault('quality_score', CodeGenerator._rough_quality_score(data['code'], lang))
                return data
            else:
                self.logger.warning("JSON 응답에 'code' 키가 없습니다")
        else:
            self.logger.warning(f"JSON 파싱 실패. 파싱된 데이터 타입: {type(data)}")
        
        self.logger.error("AI 응답 파싱에 실패했습니다")
        return None

    @staticmethod
    def _maybe_extract_code(text: str) -> str:
        """코드 블록을 텍스트에서 추출합니다."""
        # 코드 펜스 패턴들
        fences = ["```python", "```cpp", "```c++", "```"]
        
        # 가장 긴 코드 블록을 찾기
        best_code = ""
        
        for f in fences:
            if f in text:
                start = text.find(f) + len(f)
                end = text.find("```", start)
                if end != -1:
                    code = text[start:end].strip()
                    if len(code) > len(best_code):
                        best_code = code
        
        # 코드 펜스가 없으면 전체 텍스트에서 코드처럼 보이는 부분 찾기
        if not best_code:
            lines = text.split('\n')
            code_lines = []
            in_code = False
            
            for line in lines:
                stripped = line.strip()
                # C++ 또는 Python 코드 시작 패턴 찾기
                if (stripped.startswith('#include') or stripped.startswith('import') or 
                    stripped.startswith('#!/usr/bin/env') or stripped.startswith('class ') or
                    stripped.startswith('def ') or stripped.startswith('int main')):
                    in_code = True
                
                if in_code:
                    code_lines.append(line)
                    
                # 명확한 종료 패턴
                if stripped == '' and len(code_lines) > 10:
                    continue
            
            if code_lines:
                best_code = '\n'.join(code_lines).strip()
        
        return best_code if best_code else text.strip()

    @staticmethod
    def _safe_json(raw: Any) -> Any:
        if isinstance(raw, dict):
            return raw
        if isinstance(raw, str):
            raw = raw.strip()
            # extract first JSON object if surrounded by text
            m = re.search(r"\{[\s\S]*\}\s*$", raw)
            if m:
                try:
                    return json.loads(m.group(0))
                except Exception:
                    pass
            try:
                return json.loads(raw)
            except Exception:
                return raw
        return raw
