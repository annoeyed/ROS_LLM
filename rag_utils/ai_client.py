#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified AI Client
- Factory to create different provider clients (OpenAI, Anthropic, Mock)
- Automatic endpoint routing for OpenAI (Responses vs Chat) based on model
"""

import os
import json
import logging
import re
from typing import Dict, Any, Optional, Tuple
from abc import ABC, abstractmethod

# ---------------------------------------------------------------------
# Optional: load .env if python-dotenv is available (safe no-op if not)
# ---------------------------------------------------------------------
try:
    from dotenv import load_dotenv  # type: ignore
    load_dotenv()
except Exception:
    pass

# ---------------------------------------------------------------------
# Base Interface
# ---------------------------------------------------------------------
class AIClient(ABC):
    """Abstract base client for AI providers."""

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    def generate_response(self, prompt: str, **kwargs) -> str:
        """
        Generate a text response from the provider.

        Common kwargs:
          - system: Optional[str] system instruction
          - temperature: float
          - max_tokens: int
          - json_mode: bool (OpenAI Chat 전용 JSON 모드 시도)
        """
        ...

    @abstractmethod
    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        """Run a basic analysis flow built on top of generate_response."""
        ...


# ---------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------
_JSON_SNIPPET_RE = re.compile(r"\{(?:[^{}]|(?R))*\}", re.DOTALL)

def _extract_first_json_snippet(text: str) -> Optional[str]:
    """
    Extract the first top-level JSON object from a string (very loose).
    Returns None if not found.
    """
    m = _JSON_SNIPPET_RE.search(text)
    if not m:
        return None
    return m.group(0)

def _parse_json_loose(text: str) -> Tuple[Optional[Dict[str, Any]], bool]:
    """
    Try strict JSON first; if it fails, try to extract a JSON-looking snippet.
    Returns (obj, parsing_error_flag).
    """
    try:
        return json.loads(text), False
    except json.JSONDecodeError:
        snippet = _extract_first_json_snippet(text)
        if snippet:
            try:
                return json.loads(snippet), False
            except json.JSONDecodeError:
                return None, True
    return None, True


# ---------------------------------------------------------------------
# OpenAI Client (with automatic Responses/Chat routing)
# ---------------------------------------------------------------------
class OpenAIClient(AIClient):
    """
    OpenAI client with automatic routing:
      - gpt-4o / gpt-4o-mini: Chat Completions (default) or Responses
      - gpt-4.1* / gpt-5*: Responses API preferred
    Set OPENAI_MODEL and OPENAI_API_KEY in your environment.
    """

    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        super().__init__()
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        self.model = (model or os.getenv("OPENAI_MODEL", "gpt-4o-mini")).strip()

        if not self.api_key:
            self.logger.warning("OPENAI_API_KEY is not set.")

        # New OpenAI SDK import path
        try:
            from openai import OpenAI  # type: ignore
            self._OpenAI = OpenAI
            self.client = OpenAI(api_key=self.api_key)
        except Exception as e:
            self.logger.error("OpenAI SDK not available. Install with: pip install openai")
            self.logger.debug("Import error detail: %s", e)
            self.client = None

        self.logger.info(f"OpenAIClient initialized (model={self.model})")

    def _use_responses_api(self) -> bool:
        """Decide whether to call Responses API for this model."""
        m = (self.model or "").lower()
        force = os.getenv("USE_RESPONSES_API", "").lower() in ("1", "true", "yes")
        # Prefer Responses for new families; allow env override.
        return force or m.startswith(("gpt-4.1", "gpt-5"))

    def generate_response(self, prompt: str, **kwargs) -> str:
        if not self.client or not self.api_key:
            return self._fallback_response(prompt, "OpenAI client not available.")

        system = kwargs.get(
            "system",
            "You are a ROS security expert. Provide accurate and practical answers considering security."
        )
        temperature = kwargs.get("temperature", float(os.getenv("AI_TEMPERATURE", 0.3)))
        max_tokens = kwargs.get("max_tokens", int(os.getenv("AI_MAX_TOKENS", 1000)))
        json_mode = bool(kwargs.get("json_mode", False))

        try:
            if self._use_responses_api():
                # Responses API path
                r = self.client.responses.create(
                    model=self.model,
                    input=prompt if not system else f"[SYSTEM]\n{system}\n\n[USER]\n{prompt}",
                    temperature=temperature,
                    max_output_tokens=max_tokens,
                )
                return getattr(r, "output_text", None) or _extract_responses_text(r)

            # Chat Completions path
            extra = {}
            if json_mode:
                # Try JSON mode; ignore if SDK/model doesn't support
                extra["response_format"] = {"type": "json_object"}

            r = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system},
                    {"role": "user", "content": prompt},
                ],
                temperature=temperature,
                max_tokens=max_tokens,
                **extra,
            )
            return (r.choices[0].message.content or "").strip()

        except Exception as e:
            self.logger.error("OpenAI API call failed: %s", e)
            return self._fallback_response(prompt, f"API error: {e}")

    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        """
        Convenience method showing how higher-level requests can be built
        on top of generate_response, with optional JSON parsing.
        """
        if analysis_type == "security":
            prompt = f"""
Analyze the security of the following code or algorithm:

{content}

Return a STRICT JSON object with:
{{
  "risk_level": "high" | "medium" | "low",
  "risk_score": 0-100,
  "vulnerabilities": [
    {{
      "type": "string",
      "description": "string",
      "severity": "string",
      "mitigation": "string"
    }}
  ],
  "recommendations": ["string", "string"]
}}
""".strip()
            resp = self.generate_response(prompt, json_mode=True, **kwargs)
            obj, err = _parse_json_loose(resp)
            if obj is not None:
                return obj
            return {"response": resp, "type": analysis_type, "parsing_error": True}

        elif analysis_type == "code_generation":
            prompt = f"""
Generate ROS 2 Python code according to the following requirements:

{content}

Include secure design aspects:
1) input validation
2) exception handling
3) safe defaults
4) security logging

Output only code if possible.
""".strip()
            resp = self.generate_response(prompt, **kwargs)
            return {"response": resp, "type": analysis_type}

        else:
            prompt = f"Analyze the following content:\n\n{content}"
            resp = self.generate_response(prompt, **kwargs)
            return {"response": resp, "type": analysis_type}

    def _fallback_response(self, prompt: str, error_msg: str) -> str:
        return f"Unable to perform AI analysis: {error_msg}\n\nRequested prompt:\n{prompt}"


def _extract_responses_text(r: Any) -> str:
    """
    Fallback extractor for Responses API outputs if .output_text
    is not present in your SDK version.
    """
    try:
        # Common structures in recent SDKs
        if hasattr(r, "output") and r.output:
            # r.output is a list[Block]; take all text blocks concatenated
            texts = []
            for block in r.output:
                content = getattr(block, "content", None) or []
                for item in content:
                    t = getattr(item, "text", None)
                    if t:
                        texts.append(t)
            if texts:
                return "\n".join(texts)
        # Older/other fallbacks—stringify minimally
        if hasattr(r, "choices") and r.choices:
            msg = getattr(r.choices[0], "message", None)
            if msg and hasattr(msg, "content"):
                return str(msg.content)
    except Exception:
        pass
    return str(r)


# ---------------------------------------------------------------------
# Anthropic Client
# ---------------------------------------------------------------------
class AnthropicClient(AIClient):
    """
    Anthropic client wrapper (Claude).
    Set ANTHROPIC_API_KEY and ANTHROPIC_MODEL (e.g., claude-3-5-haiku-20241022).
    """

    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        super().__init__()
        self.api_key = api_key or os.getenv("ANTHROPIC_API_KEY")
        self.model = (model or os.getenv("ANTHROPIC_MODEL", "claude-3-sonnet-20240229")).strip()

        if not self.api_key:
            self.logger.warning("ANTHROPIC_API_KEY is not set.")

        try:
            import anthropic  # type: ignore
            self._anthropic = anthropic
            self.client = anthropic.Anthropic(api_key=self.api_key)
        except Exception as e:
            self.logger.error("Anthropic SDK not available. Install with: pip install anthropic")
            self.logger.debug("Import error detail: %s", e)
            self.client = None

        self.logger.info(f"AnthropicClient initialized (model={self.model})")

    def generate_response(self, prompt: str, **kwargs) -> str:
        if not self.client or not self.api_key:
            return self._fallback_response(prompt, "Anthropic client not available.")

        system = kwargs.get(
            "system",
            "You are a ROS security expert. Provide accurate and practical answers considering security."
        )
        temperature = kwargs.get("temperature", float(os.getenv("AI_TEMPERATURE", 0.3)))
        max_tokens = kwargs.get("max_tokens", int(os.getenv("AI_MAX_TOKENS", 1000)))

        try:
            r = self.client.messages.create(
                model=self.model,
                max_tokens=max_tokens,
                temperature=temperature,
                system=system,
                messages=[{"role": "user", "content": prompt}],
            )
            # Claude returns a content array; take all text blocks joined
            content = r.content or []
            texts = []
            for c in content:
                t = getattr(c, "text", None)
                if t:
                    texts.append(t)
            return "\n".join(texts).strip()
        except Exception as e:
            self.logger.error("Anthropic API call failed: %s", e)
            return self._fallback_response(prompt, f"API error: {e}")

    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        if analysis_type == "security":
            prompt = f"""
Analyze the security of the following code or algorithm:

{content}

Return a STRICT JSON object with:
{{
  "risk_level": "high" | "medium" | "low",
  "risk_score": 0-100,
  "vulnerabilities": [
    {{
      "type": "string",
      "description": "string",
      "severity": "string",
      "mitigation": "string"
    }}
  ],
  "recommendations": ["string", "string"]
}}
""".strip()
            resp = self.generate_response(prompt, **kwargs)
            obj, err = _parse_json_loose(resp)
            if obj is not None:
                return obj
            return {"response": resp, "type": analysis_type, "parsing_error": True}

        elif analysis_type == "code_generation":
            prompt = f"""
Generate ROS 2 Python code according to the following requirements:

{content}

Include secure design aspects:
1) input validation
2) exception handling
3) safe defaults
4) security logging

Output only code if possible.
""".strip()
            resp = self.generate_response(prompt, **kwargs)
            return {"response": resp, "type": analysis_type}

        else:
            prompt = f"Analyze the following content:\n\n{content}"
            resp = self.generate_response(prompt, **kwargs)
            return {"response": resp, "type": analysis_type}

    def _fallback_response(self, prompt: str, error_msg: str) -> str:
        return f"Unable to perform AI analysis: {error_msg}\n\nRequested prompt:\n{prompt}"


# ---------------------------------------------------------------------
# Mock Client (for offline/testing)
# ---------------------------------------------------------------------
class MockAIClient(AIClient):
    """Lightweight mock client for tests and local dev."""

    def __init__(self):
        super().__init__()
        self.logger.info("MockAIClient initialized (testing mode).")

    def generate_response(self, prompt: str, **kwargs) -> str:
        if "security" in prompt.lower():
            return json.dumps({
                "risk_level": "medium",
                "risk_score": 65,
                "vulnerabilities": [{
                    "type": "Input validation missing",
                    "description": "No checks on user-supplied data.",
                    "severity": "medium",
                    "mitigation": "Add explicit validation and length/charset checks."
                }],
                "recommendations": [
                    "Harden input validation",
                    "Improve exception handling",
                    "Implement security logging"
                ]
            }, indent=2)
        if "code" in prompt.lower():
            return (
                "# Mock ROS 2 node code\n"
                "import rclpy\n"
                "from rclpy.node import Node\n\n"
                "class SecureNode(Node):\n"
                "    def __init__(self):\n"
                "        super().__init__('secure_node')\n"
                "        self.get_logger().info('Secure node started (mock).')\n"
            )
        return f"Mock response: {prompt[:100]}..."

    def analyze_content(self, content: str, analysis_type: str, **kwargs) -> Dict[str, Any]:
        if analysis_type == "security":
            return {
                "risk_level": "medium",
                "risk_score": 65,
                "vulnerabilities": [{
                    "type": "Input validation missing",
                    "description": "No checks on user-supplied data.",
                    "severity": "medium",
                    "mitigation": "Add explicit validation and length/charset checks."
                }],
                "recommendations": [
                    "Harden input validation",
                    "Improve exception handling",
                    "Implement security logging"
                ]
            }
        elif analysis_type == "code_generation":
            return {
                "response": self.generate_response("code generation"),
                "type": analysis_type
            }
        return {"response": f"Mock analysis: {content[:60]}...", "type": analysis_type}


# ---------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------
class AIClientFactory:
    """Simple factory to create provider clients based on a name/env."""

    @staticmethod
    def create_client(client_type: str = "mock", **kwargs) -> AIClient:
        ctype = (client_type or os.getenv("AI_CLIENT_TYPE", "mock")).lower()
        if ctype == "openai":
            model = kwargs.get("model") or os.getenv("OPENAI_MODEL", "gpt-4o-mini")
            return OpenAIClient(model=model, api_key=kwargs.get("api_key"))
        if ctype == "anthropic":
            model = kwargs.get("model") or os.getenv("ANTHROPIC_MODEL", "claude-3-sonnet-20240229")
            return AnthropicClient(model=model, api_key=kwargs.get("api_key"))
        if ctype == "mock":
            return MockAIClient()
        logging.warning("Unknown client_type=%s. Falling back to MockAIClient.", ctype)
        return MockAIClient()


# ---------------------------------------------------------------------
# Module self-test (optional)
# ---------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Pick client from env or default to mock
    client = AIClientFactory.create_client(os.getenv("AI_CLIENT_TYPE", "mock"))

    print("\n[Security analysis demo]")
    sec = client.analyze_content("unvalidated user input flows into exec()", "security")
    print(json.dumps(sec, indent=2))

    print("\n[Code generation demo]")
    code = client.generate_response("Write a secure ROS 2 Python node skeleton.")
    print(code[:400] + ("..." if len(code) > 400 else ""))