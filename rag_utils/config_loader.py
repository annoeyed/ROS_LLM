#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configuration Loader
Utility for loading environment variables and configuration files
"""

import os
import logging
from typing import Dict, Any, Optional
from pathlib import Path

class ConfigLoader:
    """Configuration loader class"""
    
    def __init__(self, config_dir: str = "config"):
        self.config_dir = Path(config_dir)
        self.logger = logging.getLogger(self.__class__.__name__)
        
    def load_env_file(self, filename: str = ".env") -> Dict[str, str]:
        """Load environment variable file"""
        # Try root directory first, then config directory
        root_env_file = Path(filename)
        config_env_file = self.config_dir / filename
        
        env_file = root_env_file if root_env_file.exists() else config_env_file
        
        if not env_file.exists():
            self.logger.warning(f"Environment variable file not found: {env_file}")
            return {}
        
        config = {}
        try:
            with open(env_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, value = line.split('=', 1)
                        config[key.strip()] = value.strip()
            
            self.logger.info(f"Environment variable file loaded: {env_file}")
            return config
            
        except Exception as e:
            self.logger.error(f"Failed to load environment variable file: {e}")
            return {}
    
    def get_ai_config(self) -> Dict[str, Any]:
        """Get AI configuration"""
        config = {}
        
        # Load .env file first
        env_config = self.load_env_file()
        if env_config:
            config.update(env_config)
        
        # Override with direct environment variables if they exist
        if os.getenv('AI_CLIENT_TYPE'):
            config['client_type'] = os.getenv('AI_CLIENT_TYPE')
        elif 'AI_CLIENT_TYPE' in config:
            config['client_type'] = config['AI_CLIENT_TYPE']
        else:
            config['client_type'] = 'openai'  # default
        
        if os.getenv('OPENAI_API_KEY'):
            config['openai_api_key'] = os.getenv('OPENAI_API_KEY')
        elif 'OPENAI_API_KEY' in config:
            config['openai_api_key'] = config['OPENAI_API_KEY']
            
        if os.getenv('OPENAI_MODEL'):
            config['openai_model'] = os.getenv('OPENAI_MODEL')
        elif 'OPENAI_MODEL' in config:
            config['openai_model'] = config['OPENAI_MODEL']
        else:
            config['openai_model'] = 'gpt-4o-mini'
            
        if os.getenv('ANTHROPIC_API_KEY'):
            config['anthropic_api_key'] = os.getenv('ANTHROPIC_API_KEY')
        elif 'ANTHROPIC_API_KEY' in config:
            config['anthropic_api_key'] = config['ANTHROPIC_API_KEY']
            
        if os.getenv('ANTHROPIC_MODEL'):
            config['anthropic_model'] = os.getenv('ANTHROPIC_MODEL')
        elif 'ANTHROPIC_MODEL' in config:
            config['anthropic_model'] = config['ANTHROPIC_MODEL']
        else:
            config['anthropic_model'] = 'claude-3-sonnet-20240229'
            
        # Only set max_tokens if explicitly configured
        if os.getenv('AI_MAX_TOKENS'):
            config['max_tokens'] = int(os.getenv('AI_MAX_TOKENS'))
        elif 'AI_MAX_TOKENS' in config:
            config['max_tokens'] = int(config['AI_MAX_TOKENS'])
        # Remove default max_tokens - let the API use its own defaults
            
        if os.getenv('AI_TEMPERATURE'):
            config['temperature'] = float(os.getenv('AI_TEMPERATURE'))
        elif 'AI_TEMPERATURE' in config:
            config['temperature'] = float(config['AI_TEMPERATURE'])
        else:
            config['temperature'] = 0.3
        
        return config
    
    def get_model_info(self) -> Dict[str, str]:
        """Return available model information"""
        return {
            'openai': {
                'gpt-4': 'GPT-4 (Most powerful model)',
                'gpt-4-turbo': 'GPT-4 Turbo (Fast and efficient)',
                'gpt-3.5-turbo': 'GPT-3.5 Turbo (Fast and economical)',
                'gpt-4o': 'GPT-4o (Latest model)',
                'gpt-4o-mini': 'GPT-4o Mini (Economical)'
            },
            'anthropic': {
                'claude-3-opus-20240229': 'Claude 3 Opus (Most powerful model)',
                'claude-3-sonnet-20240229': 'Claude 3 Sonnet (Balanced performance)',
                'claude-3-haiku-20240307': 'Claude 3 Haiku (Fast and economical)',
                'claude-3.5-sonnet-20241022': 'Claude 3.5 Sonnet (Latest model)'
            }
        }
    
    def validate_config(self, config: Dict[str, Any]) -> bool:
        """Validate configuration"""
        required_fields = {
            'openai': ['openai_api_key'],
            'anthropic': ['anthropic_api_key'],
            'mock': []
        }
        
        client_type = config.get('client_type', 'mock')
        if client_type not in required_fields:
            self.logger.error(f"Unknown client type: {client_type}")
            return False
        
        for field in required_fields[client_type]:
            if not config.get(field):
                self.logger.error(f"Required configuration missing: {field}")
                return False
        
        return True
    
    def print_config_summary(self, config: Dict[str, Any]):
        """Print configuration summary"""
        print("\n=== AI Configuration Summary ===")
        print(f"Client Type: {config.get('client_type', 'N/A')}")
        
        if config.get('client_type') == 'openai':
            print(f"OpenAI Model: {config.get('openai_model', 'N/A')}")
            print(f"OpenAI API Key: {'Set' if config.get('openai_api_key') else 'Not set'}")
        elif config.get('client_type') == 'anthropic':
            print(f"Anthropic Model: {config.get('anthropic_model', 'N/A')}")
            print(f"Anthropic API Key: {'Set' if config.get('anthropic_api_key') else 'Not set'}")
        
        print(f"Max Tokens: {config.get('max_tokens', 'N/A')}")
        print(f"Temperature: {config.get('temperature', 'N/A')}")
        print("================================\n")

# Usage example
if __name__ == "__main__":
    config_loader = ConfigLoader()
    config = config_loader.get_ai_config()
    
    print("Loaded configuration:")
    for key, value in config.items():
        if 'key' in key.lower():
            print(f"{key}: {'Set' if value else 'Not set'}")
        else:
            print(f"{key}: {value}")
    
    print("\nAvailable models:")
    models = config_loader.get_model_info()
    for provider, model_list in models.items():
        print(f"\n{provider.upper()}:")
        for model, description in model_list.items():
            print(f"  {model}: {description}")
    
    if config_loader.validate_config(config):
        print("\nConfiguration is valid!")
        config_loader.print_config_summary(config)
    else:
        print("\nThere are issues with the configuration.")
