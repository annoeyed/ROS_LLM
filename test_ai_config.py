#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI Configuration Test Script
Tests environment variables and model configuration.
"""

import os
import sys
import logging

# Logging configuration
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# Add project root to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rag_utils.config_loader import ConfigLoader
from rag_utils.ai_client import AIClientFactory

def test_config_loader():
    """Configuration loader test"""
    print("=== Configuration Loader Test ===")
    
    config_loader = ConfigLoader()
    config = config_loader.get_ai_config()
    
    print("Current configuration:")
    for key, value in config.items():
        if 'key' in key.lower():
            print(f"  {key}: {'Set' if value else 'Not set'}")
        else:
            print(f"  {key}: {value}")
    
    print("\nAvailable models:")
    models = config_loader.get_model_info()
    for provider, model_list in models.items():
        print(f"\n{provider.upper()}:")
        for model, description in model_list.items():
            print(f"  {model}: {description}")
    
    # Configuration validation
    if config_loader.validate_config(config):
        print("\n Configuration is valid!")
        config_loader.print_config_summary(config)
    else:
        print("\n Configuration has issues.")
    
    return config

def test_ai_clients(config):
    """AI client test"""
    print("\n=== AI Client Test ===")
    
    # Mock client test
    print("\n1. Mock client test:")
    mock_client = AIClientFactory.create_client("mock")
    response = mock_client.generate_response("Please generate a ROS security node")
    print(f"   Response: {response[:100]}...")
    
    # OpenAI client test (if API key is available)
    if config.get('openai_api_key'):
        print("\n2. OpenAI client test:")
        try:
            openai_client = AIClientFactory.create_client("openai")
            print(f"   Model: {openai_client.model}")
            response = openai_client.generate_response("Simple ROS node description")
            print(f"   Response: {response[:100]}...")
        except Exception as e:
            print(f"   OpenAI client test failed: {e}")
    else:
        print("\n2. OpenAI client test: API key not set")
    
    # Anthropic client test (if API key is available)
    if config.get('anthropic_api_key'):
        print("\n3. Anthropic client test:")
        try:
            anthropic_client = AIClientFactory.create_client("anthropic")
            print(f"   Model: {anthropic_client.model}")
            response = anthropic_client.generate_response("Simple ROS node description")
            print(f"   Response: {response[:100]}...")
        except Exception as e:
            print(f"   Anthropic client test failed: {e}")
    else:
        print("\n3. Anthropic client test: API key not set")

def test_model_selection():
    """Model selection test"""
    print("\n=== Model Selection Test ===")
    
    # Test model setting via environment variables
    test_models = {
        'openai': ['gpt-4', 'gpt-4-turbo', 'gpt-3.5-turbo'],
        'anthropic': ['claude-3-sonnet-20240229', 'claude-3-haiku-20240307']
    }
    
    for provider, models in test_models.items():
        print(f"\n{provider.upper()} model test:")
        for model in models:
            try:
                # Set environment variables
                if provider == 'openai':
                    os.environ['OPENAI_MODEL'] = model
                    client = AIClientFactory.create_client("openai")
                else:
                    os.environ['ANTHROPIC_MODEL'] = model
                    client = AIClientFactory.create_client("anthropic")
                
                print(f"   {model}: Success")
                
            except Exception as e:
                print(f"   {model}: Failed - {e}")

def main():
    """Main function"""
    print("Starting AI configuration and model selection test...\n")
    
    # 1. Configuration loader test
    config = test_config_loader()
    
    # 2. AI client test
    test_ai_clients(config)
    
    # 3. Model selection test
    test_model_selection()
    
    print("\n=== Test Complete ===")
    print("\nUsage:")
    print("1. Copy config/ai_config.env.example to .env")
    print("2. Set API keys and desired models")
    print("3. Environment variable examples:")
    print("   export AI_CLIENT_TYPE=openai")
    print("   export OPENAI_API_KEY=your_key_here")
    print("   export OPENAI_MODEL=gpt-4-turbo")
    print("   export ANTHROPIC_MODEL=claude-3-haiku-20240307")

if __name__ == "__main__":
    main()
