#!/usr/bin/env python3
"""
ROS CWE 데이터베이스 구축 실행 스크립트
"""

import sys
import os

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rag_utils.cwe_collector import ROSCWECollector

def main():
    print("=== ROS CWE 데이터베이스 구축 도구 ===")
    print("이 도구는 NVD CWE API를 사용하여")
    print("ROS 관련 취약점 유형(CWE)을 수집합니다.")
    print()
    
    # NVD API 키 입력 (선택사항)
    api_key = input("NVD API 키를 입력하세요 (없으면 Enter): ").strip()
    if not api_key:
        api_key = None
        print("API 키 없이 진행합니다 (제한된 요청만 가능)")
    else:
        # .env 파일에 API 키 저장
        with open('.env', 'w') as f:
            f.write(f'NVD_API_KEY={api_key}\n')
        print(".env 파일에 API 키를 저장했습니다.")
    
    print()
    
    # 작업 선택
    print("수행할 작업을 선택하세요:")
    print("1. 새로 구축 (기존 데이터 삭제)")
    print("2. 업데이트 (기존 데이터 유지)")
    print("3. 통계 보기")
    print("4. 기존 CWE에 카테고리 할당")
    
    choice = input("선택 (1-4): ").strip()
    
    try:
        collector = ROSCWECollector()
        
        if choice == "1":
            # 백업 여부 확인
            backup_choice = input("기존 데이터를 백업하시겠습니까? (y/n): ").strip().lower()
            
            if backup_choice == 'y':
                backup_dir = collector.backup_existing_database()
                print(f"백업 완료: {backup_dir}")
                print()
            
            # 구축 진행 여부 확인
            proceed = input("데이터베이스를 새로 구축하시겠습니까? (y/n): ").strip().lower()
            
            if proceed == 'y':
                print("\n구축을 시작합니다...")
                ros_cwes = collector.build_database()
                
                print(f"\n구축 완료!")
                print(f"발견된 ROS 관련 CWE: {len(ros_cwes)}개")
                
            else:
                print("구축을 취소했습니다.")
                
        elif choice == "2":
            print("\n업데이트를 시작합니다...")
            ros_cwes = collector.update_database()
            
            print(f"\n업데이트 완료!")
            print(f"업데이트된 ROS 관련 CWE: {len(ros_cwes)}개")
            
        elif choice == "3":
            collector.show_statistics()
            
        elif choice == "4":
            print("\n기존 CWE에 카테고리를 할당합니다...")
            updated_count = collector.cwe_db.assign_categories_to_existing_cwes()
            
            if updated_count > 0:
                print(f"\n카테고리 할당 완료: {updated_count}개 CWE 업데이트")
                print("업데이트된 통계를 확인하려면 옵션 3을 선택하세요.")
            else:
                print("모든 CWE에 이미 카테고리가 할당되어 있습니다.")
            
        else:
            print("잘못된 선택입니다.")
            
    except Exception as e:
        print(f"오류 발생: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
