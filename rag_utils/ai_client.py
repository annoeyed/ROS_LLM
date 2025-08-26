#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified AI Client
- Factory to create different provider clients (OpenAI, Anthropic, etc.)
- Automatic endpoint routing for OpenAI (Responses vs Chat) based on model
"""

"""This module provides a client for interacting with AI models."""

"""This module provides a client for interacting with AI models."""

import abc
import json
import os
import regex as re
from typing import Dict, List, Optional, Union

from anthropic import Anthropic
from openai import OpenAI
from rag_utils.config import Config

# Anthropic Models
CLAUDE_3_OPUS = "claude-3-opus-20240229"
CLAUDE_3_5_SONNET = "claude-3-5-sonnet-20240620"
CLAUDE_3_HAIKU = "claude-3-haiku-20240307"

# OpenAI Models
GPT_5 = "gpt-5"
GPT_5_MINI = "gpt-5-mini"
GPT_4_O = "gpt-4o"
GPT_4O_MINI = "gpt-4o-mini"
GPT_3_5_TURBO = "gpt-3.5-turbo"


class AIClient(abc.ABC):
  """Abstract base class for an AI client."""

  @abc.abstractmethod
  def chat(self,
           prompt: str,
           history: Optional[List[Dict[str, str]]] = None,
           temperature: float = 0.0) -> str:
    """Sends a chat message to the AI model and returns the response.

    Args:
      prompt: The user's prompt.
      history: A list of previous messages in the conversation.
      temperature: The temperature for the AI model.

    Returns:
      The AI model's response.
    """
    raise NotImplementedError


# A regex to extract a code snippet from a JSON response.
# It handles nested braces.
_JSON_SNIPPET_RE = re.compile(r"\{(?:[^{}]|(?R))*\}", re.DOTALL)


def _extract_json_snippet(text: str) -> Optional[str]:
  """Extracts a JSON snippet from a given text.

  Args:
    text: The text to search for a JSON snippet.

  Returns:
    The extracted JSON snippet as a string, or None if no snippet is found.
  """
  match = _JSON_SNIPPET_RE.search(text)
  if match:
    return match.group(0)
  return None


class OpenAIClient(AIClient):
  """A client for interacting with the OpenAI API."""

  def __init__(self, model_name: str, api_key: Optional[str] = None, max_tokens: Optional[int] = None):
    """Initializes the OpenAIClient.

    Args:
      model_name: The name of the OpenAI model to use.
      api_key: The OpenAI API key.
      max_tokens: The maximum number of tokens to generate (optional).
    """
    self.model_name = model_name
    self.max_tokens = max_tokens
    self.client = OpenAI(api_key=api_key)

  def chat(self,
           prompt: str,
           history: Optional[List[Dict[str, str]]] = None,
           temperature: float = 0.0) -> str:
    """Sends a chat message to the OpenAI model and returns the response.

    Args:
      prompt: The user's prompt.
      history: A list of previous messages in the conversation.
      temperature: The temperature for the AI model.

    Returns:
      The OpenAI model's response.
    """
    messages = []
    if history:
      messages.extend(history)
    messages.append({"role": "user", "content": prompt})

    # Only include max_tokens if specified
    kwargs = {
        'model': self.model_name,
        'messages': messages,
        'temperature': temperature,
    }
    if self.max_tokens is not None:
        kwargs['max_tokens'] = self.max_tokens
    
    response = self.client.chat.completions.create(**kwargs)
    return response.choices[0].message.content


class AnthropicClient(AIClient):
  """A client for interacting with the Anthropic API."""

  def __init__(self, model_name: str, api_key: Optional[str] = None, max_tokens: Optional[int] = None):
    """Initializes the AnthropicClient.

    Args:
      model_name: The name of the Anthropic model to use.
      api_key: The Anthropic API key.
      max_tokens: The maximum number of tokens to generate (optional, defaults to 4096 for Anthropic).
    """
    self.model_name = model_name
    self.max_tokens = max_tokens or 4096  # Anthropic requires max_tokens, so use 4096 as default
    self.client = Anthropic(api_key=api_key)

  def chat(self,
           prompt: str,
           history: Optional[List[Dict[str, str]]] = None,
           temperature: float = 0.0) -> str:
    """Sends a chat message to the Anthropic model and returns the response.

    Args:
      prompt: The user's prompt.
      history: A list of previous messages in the conversation.
      temperature: The temperature for the AI model.

    Returns:
      The Anthropic model's response.
    """
    messages = []
    system_prompt = ""
    if history:
      if history[0]["role"] == "system":
        system_prompt = history[0]["content"]
        messages.extend(history[1:])
      else:
        messages.extend(history)

    messages.append({"role": "user", "content": prompt})

    response = self.client.messages.create(
        model=self.model_name,
        max_tokens=self.max_tokens,
        messages=messages,
        system=system_prompt if system_prompt else None,
        temperature=temperature,
    )
    return response.content[0].text


class AIClientFactory:
  """A factory for creating AI clients."""

  @staticmethod
  def create_client(
      client_name: str,
      config: Union[Config, Dict],
      model_name: Optional[str] = None) -> Optional[AIClient]:
    """Creates an AI client based on the provided name and configuration.

    Args:
      client_name: The name of the client to create (e.g., "openai",
        "anthropic").
      config: The configuration object or dictionary containing API keys and model names.
      model_name: The specific model name to use, overriding the default in the
        config.

    Returns:
      An instance of the specified AI client, or None if the client name is
      unknown.
    """
    # Handle both Config object and dictionary
    if isinstance(config, dict):
      max_tokens = config.get('max_tokens')  # Only use if explicitly set
      if client_name == "openai":
        model = model_name or config.get('openai_model', 'gpt-4')
        api_key = config.get('openai_api_key')
        if not api_key:
          print("OpenAI API key not found in configuration.")
          return None
        return OpenAIClient(model_name=model, api_key=api_key, max_tokens=max_tokens)
      elif client_name == "anthropic":
        model = model_name or config.get('anthropic_model', 'claude-3-sonnet-20240229')
        api_key = config.get('anthropic_api_key')
        if not api_key:
          print("Anthropic API key not found in configuration.")
          return None
        return AnthropicClient(model_name=model, api_key=api_key, max_tokens=max_tokens)
      else:
        print(f"Unknown client: {client_name}. AI Client creation failed.")
        return None
    else:
      # Original Config object handling
      max_tokens = getattr(config, 'max_tokens', None)
      if client_name == "openai":
        model = model_name or config.model.name
        api_key = config.api_keys.openai
        return OpenAIClient(model_name=model, api_key=api_key, max_tokens=max_tokens)
      elif client_name == "anthropic":
        model = model_name or config.model.name
        api_key = config.api_keys.anthropic
        return AnthropicClient(model_name=model, api_key=api_key, max_tokens=max_tokens)
      else:
        print(f"Unknown client: {client_name}. AI Client creation failed.")
        return None

