#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
설정 로더
환경 변수와 설정 파일을 로드하는 유틸리티
"""

import os
import logging
from typing import Dict, Any, Optional
from pathlib import Path

class ConfigLoader:
    """설정 로더 클래스"""
    
    def __init__(self, config_dir: str = "config"):
        self.config_dir = Path(config_dir)
        self.logger = logging.getLogger(self.__class__.__name__)
        
    def load_env_file(self, filename: str = ".env") -> Dict[str, str]:
        """환경 변수 파일 로드"""
        env_file = self.config_dir / filename
        
        if not env_file.exists():
            self.logger.warning(f"환경 변수 파일을 찾을 수 없습니다: {env_file}")
            return {}
        
        config = {}
        try:
            with open(env_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, value = line.split('=', 1)
                        config[key.strip()] = value.strip()
            
            self.logger.info(f"환경 변수 파일 로드 완료: {env_file}")
            return config
            
        except Exception as e:
            self.logger.error(f"환경 변수 파일 로드 실패: {e}")
            return {}
    
    def get_ai_config(self) -> Dict[str, Any]:
        """AI 설정 가져오기"""
        config = {}
        
        # 환경 변수에서 직접 설정
        config['client_type'] = os.getenv('AI_CLIENT_TYPE', 'mock')
        config['openai_api_key'] = os.getenv('OPENAI_API_KEY')
        config['openai_model'] = os.getenv('OPENAI_MODEL', 'gpt-4')
        config['anthropic_api_key'] = os.getenv('ANTHROPIC_API_KEY')
        config['anthropic_model'] = os.getenv('ANTHROPIC_MODEL', 'claude-3-sonnet-20240229')
        config['max_tokens'] = int(os.getenv('AI_MAX_TOKENS', '1000'))
        config['temperature'] = float(os.getenv('AI_TEMPERATURE', '0.3'))
        
        # .env 파일이 있다면 로드
        env_config = self.load_env_file()
        if env_config:
            config.update(env_config)
        
        return config
    
    def get_model_info(self) -> Dict[str, str]:
        """사용 가능한 모델 정보 반환"""
        return {
            'openai': {
                'gpt-4': 'GPT-4 (가장 강력한 모델)',
                'gpt-4-turbo': 'GPT-4 Turbo (빠르고 효율적)',
                'gpt-3.5-turbo': 'GPT-3.5 Turbo (빠르고 경제적)',
                'gpt-4o': 'GPT-4o (최신 모델)',
                'gpt-4o-mini': 'GPT-4o Mini (경제적)'
            },
            'anthropic': {
                'claude-3-opus-20240229': 'Claude 3 Opus (가장 강력한 모델)',
                'claude-3-sonnet-20240229': 'Claude 3 Sonnet (균형잡힌 성능)',
                'claude-3-haiku-20240307': 'Claude 3 Haiku (빠르고 경제적)',
                'claude-3.5-sonnet-20241022': 'Claude 3.5 Sonnet (최신 모델)'
            }
        }
    
    def validate_config(self, config: Dict[str, Any]) -> bool:
        """설정 유효성 검사"""
        required_fields = {
            'openai': ['openai_api_key'],
            'anthropic': ['anthropic_api_key'],
            'mock': []
        }
        
        client_type = config.get('client_type', 'mock')
        if client_type not in required_fields:
            self.logger.error(f"알 수 없는 클라이언트 타입: {client_type}")
            return False
        
        for field in required_fields[client_type]:
            if not config.get(field):
                self.logger.error(f"필수 설정이 누락되었습니다: {field}")
                return False
        
        return True
    
    def print_config_summary(self, config: Dict[str, Any]):
        """설정 요약 출력"""
        print("\n=== AI 설정 요약 ===")
        print(f"클라이언트 타입: {config.get('client_type', 'N/A')}")
        
        if config.get('client_type') == 'openai':
            print(f"OpenAI 모델: {config.get('openai_model', 'N/A')}")
            print(f"OpenAI API 키: {'설정됨' if config.get('openai_api_key') else '설정되지 않음'}")
        elif config.get('client_type') == 'anthropic':
            print(f"Anthropic 모델: {config.get('anthropic_model', 'N/A')}")
            print(f"Anthropic API 키: {'설정됨' if config.get('anthropic_api_key') else '설정되지 않음'}")
        
        print(f"최대 토큰: {config.get('max_tokens', 'N/A')}")
        print(f"온도: {config.get('temperature', 'N/A')}")
        print("==================\n")

# 사용 예시
if __name__ == "__main__":
    config_loader = ConfigLoader()
    config = config_loader.get_ai_config()
    
    print("로드된 설정:")
    for key, value in config.items():
        if 'key' in key.lower():
            print(f"{key}: {'설정됨' if value else '설정되지 않음'}")
        else:
            print(f"{key}: {value}")
    
    print("\n사용 가능한 모델:")
    models = config_loader.get_model_info()
    for provider, model_list in models.items():
        print(f"\n{provider.upper()}:")
        for model, description in model_list.items():
            print(f"  {model}: {description}")
    
    if config_loader.validate_config(config):
        print("\n설정이 유효합니다!")
        config_loader.print_config_summary(config)
    else:
        print("\n설정에 문제가 있습니다.")
