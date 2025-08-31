# rag_utils/cwe_collector.py
import os
import shutil
from datetime import datetime
from typing import List, Dict, Any, Optional
import time

from .mitre_cwe_api import MITRECWEAPIClient
from .cwe_database import CWEDatabase
from .config import Config

class ROSCWECollector:
    """ROS-related CWE collector"""
    
    def __init__(self):
        self.cwe_api = MITRECWEAPIClient()
        self.cwe_db = CWEDatabase()
        
    def backup_existing_database(self):
        """Backup existing database"""
        if os.path.exists(Config.CWE_DB_PATH):
            backup_dir = f"data/cwe_database_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            shutil.copytree(Config.CWE_DB_PATH, backup_dir)
            print(f"Existing database backed up to {backup_dir}.")
            return backup_dir
        return None
    
    def clear_existing_database(self):
        """Initialize existing database"""
        self.cwe_db.clear_database()
        print("Existing database initialized.")
    
    def collect_ros_cwes(self) -> List[Dict[str, Any]]:
        """Collect ROS-related CWEs (maintain existing method name)"""
        return self.collect_all_ros_cwes()
    
    def _assign_categories_to_cwe(self, cwe_data: Dict[str, Any]) -> Dict[str, Any]:
        """Automatically assign categories to CWE"""
        cwe_id = cwe_data.get('cweId', '')
        name = cwe_data.get('name', '').lower()
        description = cwe_data.get('description', '').lower()
        
        # Automatically assign based on configured category mapping
        assigned_category = None
        assigned_component = None
        
        # Check category-based mapping
        for category, cwe_ids in Config.ROS_CWE_CATEGORIES.items():
            if cwe_id in cwe_ids:
                assigned_category = category
                break
        
        # Check component-based mapping
        for component, cwe_ids in Config.ROS_COMPONENT_CWE_MAPPING.items():
            if cwe_id in cwe_ids:
                assigned_component = component
                break
        
        # Keyword-based automatic classification (when category is not assigned)
        if not assigned_category:
            if any(kw in name or kw in description for kw in ['authentication', 'auth', 'login', 'password']):
                assigned_category = 'authentication'
            elif any(kw in name or kw in description for kw in ['authorization', 'permission', 'access control', 'privilege']):
                assigned_category = 'authorization'
            elif any(kw in name or kw in description for kw in ['input', 'validation', 'sanitize', 'filter']):
                assigned_category = 'input_validation'
            elif any(kw in name or kw in description for kw in ['memory', 'buffer', 'overflow', 'underflow']):
                assigned_category = 'memory_management'
            elif any(kw in name or kw in description for kw in ['file', 'path', 'traversal', 'directory']):
                assigned_category = 'file_operations'
            elif any(kw in name or kw in description for kw in ['race', 'condition', 'concurrent', 'thread']):
                assigned_category = 'race_conditions'
            elif any(kw in name or kw in description for kw in ['network', 'communication', 'protocol']):
                assigned_category = 'network_security'
            elif any(kw in name or kw in description for kw in ['cryptography', 'encryption', 'hash', 'ssl', 'tls']):
                assigned_category = 'cryptography'
            elif any(kw in name or kw in description for kw in ['log', 'logging', 'audit']):
                assigned_category = 'logging'
            elif any(kw in name or kw in description for kw in ['error', 'exception', 'handling']):
                assigned_category = 'error_handling'
        
        # Add assigned category and component to CWE data
        cwe_data['category'] = assigned_category
        cwe_data['ros_component'] = assigned_component
        
        return cwe_data

    def collect_all_ros_cwes(self) -> List[Dict[str, Any]]:
        """Collect all ROS-related CWEs"""
        print("Starting ROS-related CWE collection...")
        
        all_cwes = []
        
        # 1. Direct collection from ROS_CWE_IDS (priority)
        print("\n1. Collecting CWEs from ROS_CWE_IDS...")
        for cwe_id in Config.ROS_CWE_IDS:
            cwe_data = self.search_cwe_by_id(cwe_id)
            if cwe_data:
                # Automatic category assignment
                cwe_data = self._assign_categories_to_cwe(cwe_data)
                cwe_data['source'] = 'ROS_CWE_IDS'
                all_cwes.append(cwe_data)
                print(f"  ✓ {cwe_id} 수집 완료 (카테고리: {cwe_data.get('category', 'N/A')})")
            else:
                print(f"  ✗ {cwe_id} 수집 실패")
            
            # 요청 지연
            time.sleep(Config.NVD_REQUEST_DELAY)
        
        print(f"ROS_CWE_IDS에서 {len(all_cwes)}개 CWE 수집 완료")
        
        # 2. 카테고리별 CWE 수집 (보완)
        print("\n2. 카테고리별 CWE 수집 중...")
        for category in Config.ROS_CWE_CATEGORIES.keys():
            category_cwes = self.search_cwes_by_category(category)
            # 카테고리 할당
            for cwe in category_cwes:
                cwe = self._assign_categories_to_cwe(cwe)
            all_cwes.extend(category_cwes)
        
        # 3. 컴포넌트별 CWE 수집 (보완)
        print("\n3. 컴포넌트별 CWE 수집 중...")
        for component in Config.ROS_COMPONENT_CWE_MAPPING.keys():
            component_cwes = self.search_ros_component_cwes(component)
            # 컴포넌트 할당
            for cwe in component_cwes:
                cwe = self._assign_categories_to_cwe(cwe)
            all_cwes.extend(component_cwes)
        
        # 4. 중복 제거
        unique_cwes = []
        seen_ids = set()
        
        for cwe in all_cwes:
            cwe_id = cwe.get('cweId', 'Unknown')
            if cwe_id not in seen_ids:
                unique_cwes.append(cwe)
                seen_ids.add(cwe_id)
        
        print(f"\n총 {len(unique_cwes)}개의 고유한 ROS 관련 CWE 발견!")
        
        # 카테고리별 통계 출력
        category_stats = {}
        for cwe in unique_cwes:
            category = cwe.get('category', 'uncategorized')
            if category is None:
                category = 'uncategorized'
            category_stats[category] = category_stats.get(category, 0) + 1
        
        print("\n=== 카테고리별 분류 통계 ===")
        if category_stats:
            # None 값이 있는지 확인하고 제거
            safe_stats = {k: v for k, v in category_stats.items() if k is not None}
            if safe_stats:
                for category, count in sorted(safe_stats.items()):
                    print(f"{category}: {count}개")
            else:
                print("유효한 카테고리 정보가 없습니다.")
        else:
            print("카테고리 정보가 없습니다.")
        
        return unique_cwes
    
    def validate_ros_relevance(self, cwe_data: Dict[str, Any]) -> bool:
        """CWE가 ROS와 관련있는지 검증"""
        description = cwe_data.get('description', '').lower()
        name = cwe_data.get('name', '').lower()
        
        # ROS 관련 핵심 용어들
        ros_indicators = [
            'robot', 'autonomous', 'sensor', 'actuator', 'control system',
            'real-time', 'embedded', 'network', 'communication', 'authentication',
            'authorization', 'input validation', 'memory management', 'serialization'
        ]
        
        # 하나라도 포함되어 있으면 ROS 관련으로 판단
        return any(indicator in description or indicator in name for indicator in ros_indicators)
    
    def filter_ros_cwes(self, cwes: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """ROS 관련 CWE만 필터링"""
        print("\n=== ROS 관련 CWE 필터링 중 ===")
        
        ros_cwes = []
        for cwe in cwes:
            if self.validate_ros_relevance(cwe):
                ros_cwes.append(cwe)
        
        print(f"필터링 결과: {len(ros_cwes)}/{len(cwes)}개가 ROS 관련 CWE")
        return ros_cwes
    
    def build_database(self):
        """CWE 데이터베이스 구축"""
        print("=== ROS CWE 데이터베이스 구축 시작 ===")
        
        # 1. 기존 데이터 백업
        backup_dir = self.backup_existing_database()
        
        # 2. 기존 데이터 초기화
        self.clear_existing_database()
        
        # 3. ROS 관련 CWE 수집
        all_cwes = self.collect_ros_cwes()
        
        if not all_cwes:
            print("CWE 수집에 실패했습니다.")
            return []
        
        # 4. 키워드 필터링 제거 - 모든 수집된 CWE를 ROS 관련으로 간주
        print(f"\n수집된 {len(all_cwes)}개 CWE를 모두 ROS 관련으로 간주합니다.")
        ros_cwes = all_cwes  # 필터링 없이 모든 CWE 사용
        
        # 5. 데이터베이스에 추가
        if ros_cwes:
            result = self.cwe_db.add_cwes(ros_cwes, "NVD CWE API")
            print(f"\n데이터베이스에 추가됨: {result}")
        
        # 6. RAG용 데이터 내보내기
        self.cwe_db.export_to_markdown()
        self.cwe_db.export_for_rag()
        
        print(f"\n=== 구축 완료 ===")
        if backup_dir:
            print(f"백업 위치: {backup_dir}")
        print(f"새로운 ROS CWE 개수: {len(ros_cwes)}")
        
        return ros_cwes
    
    def update_database(self):
        """기존 데이터베이스 업데이트"""
        print("=== ROS CWE 데이터베이스 업데이트 시작 ===")
        
        # 1. 새로운 CWE 수집
        new_cwes = self.collect_ros_cwes()
        
        if not new_cwes:
            print("새로운 CWE 수집에 실패했습니다.")
            return []
        
        # 2. 키워드 필터링 제거 - 모든 수집된 CWE를 ROS 관련으로 간주
        print(f"\n수집된 {len(new_cwes)}개 CWE를 모두 ROS 관련으로 간주합니다.")
        ros_cwes = new_cwes  # 필터링 없이 모든 CWE 사용
        
        # 3. 데이터베이스에 추가/업데이트
        if ros_cwes:
            result = self.cwe_db.add_cwes(ros_cwes, "NVD CWE API (업데이트)")
            print(f"\n데이터베이스 업데이트 완료: {result}")
        
        # 4. RAG용 데이터 재생성
        self.cwe_db.export_to_markdown()
        self.cwe_db.export_for_rag()
        
        print(f"\n=== 업데이트 완료 ===")
        print(f"업데이트된 ROS CWE 개수: {len(ros_cwes)}")
        
        return ros_cwes
    
    def show_statistics(self):
        """데이터베이스 통계 표시"""
        stats = self.cwe_db.get_statistics()
        
        print("=== ROS CWE 데이터베이스 통계 ===")
        print(f"총 CWE 개수: {stats.get('total_unique', 0)}")
        
        print("\n카테고리별 분류:")
        for category, count in stats.get('by_category', {}).items():
            print(f"  - {category}: {count}개")
        
        print("\nROS 컴포넌트별 분류:")
        for component, count in stats.get('by_component', {}).items():
            print(f"  - {component}: {count}개")
        
        print(f"\n마지막 업데이트: {self.cwe_db.metadata['last_updated']}")

    def search_cwe_by_id(self, cwe_id: str) -> Optional[Dict[str, Any]]:
        """특정 CWE ID로 검색"""
        return self.cwe_api.search_cwe_by_id(cwe_id)
    
    def search_cwes_by_category(self, category: str) -> List[Dict[str, Any]]:
        """카테고리별 CWE 검색"""
        cwe_ids = Config.ROS_CWE_CATEGORIES.get(category, [])
        cwes = []
        
        print(f"{category} 카테고리 CWE 검색 중...")
        
        for cwe_id in cwe_ids:
            cwe_data = self.search_cwe_by_id(cwe_id)
            if cwe_data:
                cwe_data['category'] = category
                cwes.append(cwe_data)
            
            # 요청 지연
            time.sleep(Config.NVD_REQUEST_DELAY)
        
        print(f"{category} 카테고리에서 {len(cwes)}개 CWE 발견")
        return cwes
    
    def search_ros_component_cwes(self, component: str) -> List[Dict[str, Any]]:
        """ROS 컴포넌트별 관련 CWE 검색"""
        cwe_ids = Config.ROS_COMPONENT_CWE_MAPPING.get(component, [])
        cwes = []
        
        print(f"{component} 컴포넌트 CWE 검색 중...")
        
        for cwe_id in cwe_ids:
            cwe_data = self.search_cwe_by_id(cwe_id)
            if cwe_data:
                cwe_data['ros_component'] = component
                cwes.append(cwe_data)
            
            time.sleep(Config.NVD_REQUEST_DELAY)
        
        print(f"{component} 컴포넌트에서 {len(cwes)}개 CWE 발견")
        return cwes

def main():
    """메인 실행 함수"""
    import argparse
    
    parser = argparse.ArgumentParser(description="ROS CWE 데이터베이스 구축/관리")
    parser.add_argument("--action", choices=["build", "update", "stats"], 
                       default="build", help="수행할 작업")
    parser.add_argument("--backup", action="store_true", help="백업 수행")
    
    args = parser.parse_args()
    
    collector = ROSCWECollector()
    
    if args.action == "build":
        if args.backup:
            backup_dir = collector.backup_existing_database()
            print(f"백업 완료: {backup_dir}")
        
        collector.build_database()
        
    elif args.action == "update":
        collector.update_database()
        
    elif args.action == "stats":
        collector.show_statistics()

if __name__ == "__main__":
    main()
