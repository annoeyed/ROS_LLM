#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CWE 카테고리 할당 테스트 스크립트
"""

import sys
import os

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rag_utils.cwe_database import CWEDatabase

def test_category_assignment():
    """카테고리 할당 테스트"""
    print("=== CWE 카테고리 할당 테스트 ===")
    
    try:
        # CWE 데이터베이스 로드
        cwe_db = CWEDatabase()
        print(f"데이터베이스 로드 완료: {len(cwe_db.cwes)}개 CWE")
        
        # 기존 통계 확인
        print("\n=== 기존 통계 ===")
        old_stats = cwe_db.get_statistics()
        print(f"카테고리별 분류: {old_stats.get('by_category', {})}")
        print(f"컴포넌트별 분류: {old_stats.get('by_component', {})}")
        
        # 카테고리 할당 실행
        print("\n=== 카테고리 할당 실행 ===")
        updated_count = cwe_db.assign_categories_to_existing_cwes()
        
        # 업데이트된 통계 확인
        print("\n=== 업데이트된 통계 ===")
        new_stats = cwe_db.get_statistics()
        print(f"카테고리별 분류: {new_stats.get('by_category', {})}")
        print(f"컴포넌트별 분류: {new_stats.get('by_component', {})}")
        
        # 샘플 CWE 확인
        print("\n=== 샘플 CWE 확인 ===")
        sample_cwes = cwe_db.cwes[:5]  # 처음 5개
        for cwe in sample_cwes:
            print(f"CWE-{cwe['cwe_id']}: {cwe['name']}")
            print(f"  카테고리: {cwe.get('category', 'N/A')}")
            print(f"  ROS 컴포넌트: {cwe.get('ros_component', 'N/A')}")
            print()
        
        print(f"테스트 완료! {updated_count}개 CWE 업데이트됨")
        
    except Exception as e:
        print(f"테스트 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_category_assignment()
