# rag_utils/cwe_database.py
import json
import os
from datetime import datetime
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, asdict

from .config import Config

@dataclass
class CWEData:
    """CWE data structure"""
    cwe_id: str
    name: str
    description: str
    category: Optional[str]
    ros_component: Optional[str]
    likelihood: Optional[str]
    status: str
    created_date: str
    last_modified: str
    raw_data: Dict[str, Any]

class CWEDatabase:
    """CWE database management"""
    
    def __init__(self, db_path: str = None):
        self.db_path = db_path or Config.CWE_DB_PATH
        self.cwes_file = os.path.join(self.db_path, "cwes.json")
        self.metadata_file = os.path.join(self.db_path, "metadata.json")
        
        os.makedirs(self.db_path, exist_ok=True)
        self.load_database()
    
    def load_database(self):
        """Load existing CWE database"""
        if os.path.exists(self.cwes_file):
            with open(self.cwes_file, 'r', encoding='utf-8') as f:
                self.cwes = json.load(f)
        else:
            self.cwes = []
        
        if os.path.exists(self.metadata_file):
            with open(self.metadata_file, 'r', encoding='utf-8') as f:
                self.metadata = json.load(f)
        else:
            self.metadata = {
                "last_updated": datetime.now().isoformat(),
                "total_cwes": 0,
                "sources": ["NVD CWE API"],
                "statistics": {}
            }
    
    def add_cwes(self, new_cwes: List[Dict[str, Any]], source: str = "NVD CWE API"):
        """Add new CWEs"""
        added_count = 0
        updated_count = 0
        
        for cwe_data in new_cwes:
            cwe_id = cwe_data.get('cweId', 'Unknown')
            
            # Check if CWE already exists
            existing_cwe = next((c for c in self.cwes if c['cwe_id'] == cwe_id), None)
            
            if existing_cwe:
                # Update existing CWE
                existing_cwe.update(self._extract_cwe_info(cwe_data))
                updated_count += 1
            else:
                # Add new CWE
                cwe_info = self._extract_cwe_info(cwe_data)
                self.cwes.append(cwe_info)
                added_count += 1
        
        # Update metadata
        self.metadata["last_updated"] = datetime.now().isoformat()
        self.metadata["total_cwes"] = len(self.cwes)
        if source not in self.metadata["sources"]:
            self.metadata["sources"].append(source)
        
        self._update_statistics()
        self.save_database()
        
        return {"added": added_count, "updated": updated_count}
    
    def _extract_cwe_info(self, cwe_data: Dict[str, Any]) -> Dict[str, Any]:
        """Extract necessary information from CWE data"""
        return {
            "cwe_id": cwe_data.get('cweId', 'Unknown'),
            "name": cwe_data.get('name', 'No name'),
            "description": cwe_data.get('description', 'No description'),
            "category": cwe_data.get('category'),
            "ros_component": cwe_data.get('ros_component'),
            "likelihood": cwe_data.get('likelihood'),
            "status": cwe_data.get('status', 'Incomplete'),
            "created_date": cwe_data.get('dateCreated', datetime.now().isoformat()),
            "last_modified": cwe_data.get('dateLastModified', datetime.now().isoformat()),
            "raw_data": cwe_data
        }
    
    def assign_categories_to_existing_cwes(self):
        """기존 CWE 데이터에 카테고리 자동 할당"""
        print("기존 CWE 데이터에 카테고리 자동 할당 중...")
        
        updated_count = 0
        
        for cwe in self.cwes:
            if not cwe.get('category'):  # 카테고리가 없는 경우만
                cwe_id = cwe['cwe_id']
                name = cwe['name'].lower()
                description = cwe['description'].lower()
                
                # 설정된 카테고리 매핑에 따라 자동 할당
                assigned_category = None
                assigned_component = None
                
                # 카테고리별 매핑 확인
                for category, cwe_ids in Config.ROS_CWE_CATEGORIES.items():
                    if cwe_id in cwe_ids:
                        assigned_category = category
                        break
                
                # 컴포넌트별 매핑 확인
                for component, cwe_ids in Config.ROS_COMPONENT_CWE_MAPPING.items():
                    if cwe_id in cwe_ids:
                        assigned_component = component
                        break
                
                # 키워드 기반 자동 분류 (카테고리가 할당되지 않은 경우)
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
                
                # 할당된 카테고리와 컴포넌트를 CWE 데이터에 추가
                if assigned_category:
                    cwe['category'] = assigned_category
                    updated_count += 1
                
                if assigned_component:
                    cwe['ros_component'] = assigned_component
        
        if updated_count > 0:
            self._update_statistics()
            self.save_database()
            print(f"카테고리 할당 완료: {updated_count}개 CWE 업데이트")
        else:
            print("업데이트할 CWE가 없습니다.")
        
        return updated_count

    def _update_statistics(self):
        """통계 정보 업데이트"""
        # 카테고리별 통계
        categories = {}
        components = {}
        
        for cwe in self.cwes:
            if cwe.get('category'):
                cat = cwe['category']
                categories[cat] = categories.get(cat, 0) + 1
            
            if cwe.get('ros_component'):
                comp = cwe['ros_component']
                components[comp] = components.get(comp, 0) + 1
        
        self.metadata["statistics"] = {
            "by_category": categories,
            "by_component": components,
            "total_unique": len(self.cwes)
        }
    
    def save_database(self):
        """데이터베이스 저장"""
        with open(self.cwes_file, 'w', encoding='utf-8') as f:
            json.dump(self.cwes, f, ensure_ascii=False, indent=2)
        
        with open(self.metadata_file, 'w', encoding='utf-8') as f:
            json.dump(self.metadata, f, ensure_ascii=False, indent=2)
    
    def search_cwes(self, query: str = None, category: str = None, component: str = None) -> List[Dict[str, Any]]:
        """CWE 검색"""
        results = self.cwes
        
        if query:
            query_lower = query.lower()
            results = [cwe for cwe in results if 
                      query_lower in cwe['description'].lower() or 
                      query_lower in cwe['name'].lower() or
                      query_lower in cwe['cwe_id'].lower()]
        
        if category:
            results = [cwe for cwe in results if cwe.get('category') == category]
        
        if component:
            results = [cwe for cwe in results if cwe.get('ros_component') == component]
        
        return results
    
    def get_statistics(self) -> Dict[str, Any]:
        """통계 정보 반환"""
        if "statistics" not in self.metadata or self.metadata["statistics"] is None:
            # 통계가 없으면 기본값 반환
            return {
                "total_unique": len(self.cwes),
                "by_category": {},
                "by_component": {}
            }
        return self.metadata["statistics"]
    
    def export_to_markdown(self, output_path: str = None):
        """CWE 데이터를 RAG용 마크다운으로 내보내기"""
        if output_path is None:
            output_path = os.path.join(Config.RAG_SOURCES_PATH, "ros_cwe_summary.md")
        
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(f"# ROS / ROS 2 관련 CWE 목록 (자동 수집)\n\n")
            f.write(f"**마지막 업데이트**: {self.metadata['last_updated']}\n")
            f.write(f"**총 CWE 개수**: {self.metadata['total_cwes']}\n\n")
            
            # 통계
            f.write("## 통계\n\n")
            f.write("### 카테고리별 분류\n")
            for category, count in self.metadata["statistics"]["by_category"].items():
                f.write(f"- {category}: {count}개\n")
            
            f.write("\n### ROS 컴포넌트별 분류\n")
            for component, count in self.metadata["statistics"]["by_component"].items():
                f.write(f"- {component}: {count}개\n")
            
            f.write("\n## CWE 상세 목록\n\n")
            for cwe in self.cwes:
                f.write(f"### {cwe['cwe_id']} - {cwe['name']}\n")
                f.write(f"**설명**: {cwe['description']}\n")
                if cwe.get('category'):
                    f.write(f"**카테고리**: {cwe['category']}\n")
                if cwe.get('ros_component'):
                    f.write(f"**ROS 컴포넌트**: {cwe['ros_component']}\n")
                f.write(f"**상태**: {cwe['status']}\n")
                f.write("\n---\n\n")
        
        print(f"CWE 데이터를 {output_path}로 내보냈습니다.")
    
    def export_for_rag(self, output_path: str = None):
        """RAG 시스템용 텍스트 파일 생성"""
        if output_path is None:
            output_path = os.path.join(Config.RAG_SOURCES_PATH, "ros_cwe_rag.txt")
        
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            for cwe in self.cwes:
                # RAG용 텍스트 생성
                rag_text = f"""CWE ID: {cwe['cwe_id']}
Name: {cwe['name']}
Description: {cwe['description']}
Category: {cwe.get('category', 'N/A')}
ROS Component: {cwe.get('ros_component', 'N/A')}
Status: {cwe['status']}

"""
                f.write(rag_text)
        
        print(f"RAG용 CWE 데이터를 {output_path}로 내보냈습니다.")
    
    def clear_database(self):
        """데이터베이스 초기화"""
        self.cwes = []
        self.metadata = {
            "last_updated": datetime.now().isoformat(),
            "total_cwes": 0,
            "sources": ["NVD CWE API (재구축)"],
            "statistics": {}
        }
        self.save_database()
        print("CWE 데이터베이스를 초기화했습니다.")
