#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 설정 테스트 스크립트
환경 변수와 모델 설정을 테스트합니다.
"""

import os
import sys
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# 프로젝트 루트를 Python 경로에 추가
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rag_utils.config_loader import ConfigLoader
from rag_utils.ai_client import AIClientFactory

def test_config_loader():
    """설정 로더 테스트"""
    print("=== 설정 로더 테스트 ===")
    
    config_loader = ConfigLoader()
    config = config_loader.get_ai_config()
    
    print("현재 설정:")
    for key, value in config.items():
        if 'key' in key.lower():
            print(f"  {key}: {'설정됨' if value else '설정되지 않음'}")
        else:
            print(f"  {key}: {value}")
    
    print("\n사용 가능한 모델:")
    models = config_loader.get_model_info()
    for provider, model_list in models.items():
        print(f"\n{provider.upper()}:")
        for model, description in model_list.items():
            print(f"  {model}: {description}")
    
    # 설정 유효성 검사
    if config_loader.validate_config(config):
        print("\n 설정이 유효합니다!")
        config_loader.print_config_summary(config)
    else:
        print("\n 설정에 문제가 있습니다.")
    
    return config

def test_ai_clients(config):
    """AI 클라이언트 테스트"""
    print("\n=== AI 클라이언트 테스트 ===")
    
    # Mock 클라이언트 테스트
    print("\n1. Mock 클라이언트 테스트:")
    mock_client = AIClientFactory.create_client("mock")
    response = mock_client.generate_response("ROS 보안 노드를 생성해주세요")
    print(f"   응답: {response[:100]}...")
    
    # OpenAI 클라이언트 테스트 (API 키가 있는 경우)
    if config.get('openai_api_key'):
        print("\n2. OpenAI 클라이언트 테스트:")
        try:
            openai_client = AIClientFactory.create_client("openai")
            print(f"   모델: {openai_client.model}")
            response = openai_client.generate_response("간단한 ROS 노드 설명")
            print(f"   응답: {response[:100]}...")
        except Exception as e:
            print(f"   OpenAI 클라이언트 테스트 실패: {e}")
    else:
        print("\n2. OpenAI 클라이언트 테스트: API 키가 설정되지 않음")
    
    # Anthropic 클라이언트 테스트 (API 키가 있는 경우)
    if config.get('anthropic_api_key'):
        print("\n3. Anthropic 클라이언트 테스트:")
        try:
            anthropic_client = AIClientFactory.create_client("anthropic")
            print(f"   모델: {anthropic_client.model}")
            response = anthropic_client.generate_response("간단한 ROS 노드 설명")
            print(f"   응답: {response[:100]}...")
        except Exception as e:
            print(f"   Anthropic 클라이언트 테스트 실패: {e}")
    else:
        print("\n3. Anthropic 클라이언트 테스트: API 키가 설정되지 않음")

def test_model_selection():
    """모델 선택 테스트"""
    print("\n=== 모델 선택 테스트 ===")
    
    # 환경 변수로 모델 설정 테스트
    test_models = {
        'openai': ['gpt-4', 'gpt-4-turbo', 'gpt-3.5-turbo'],
        'anthropic': ['claude-3-sonnet-20240229', 'claude-3-haiku-20240307']
    }
    
    for provider, models in test_models.items():
        print(f"\n{provider.upper()} 모델 테스트:")
        for model in models:
            try:
                # 환경 변수 설정
                if provider == 'openai':
                    os.environ['OPENAI_MODEL'] = model
                    client = AIClientFactory.create_client("openai")
                else:
                    os.environ['ANTHROPIC_MODEL'] = model
                    client = AIClientFactory.create_client("anthropic")
                
                print(f"   {model}: 성공")
                
            except Exception as e:
                print(f"   {model}: 실패 - {e}")

def main():
    """메인 함수"""
    print("AI 설정 및 모델 선택 테스트를 시작합니다...\n")
    
    # 1. 설정 로더 테스트
    config = test_config_loader()
    
    # 2. AI 클라이언트 테스트
    test_ai_clients(config)
    
    # 3. 모델 선택 테스트
    test_model_selection()
    
    print("\n=== 테스트 완료 ===")
    print("\n사용 방법:")
    print("1. config/ai_config.env.example 파일을 .env로 복사")
    print("2. API 키와 원하는 모델을 설정")
    print("3. 환경 변수 예시:")
    print("   export AI_CLIENT_TYPE=openai")
    print("   export OPENAI_API_KEY=your_key_here")
    print("   export OPENAI_MODEL=gpt-4-turbo")
    print("   export ANTHROPIC_MODEL=claude-3-haiku-20240307")

if __name__ == "__main__":
    main()
