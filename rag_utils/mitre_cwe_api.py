# rag_utils/mitre_cwe_api.py
import requests
import xml.etree.ElementTree as ET
import zipfile
import io
import os
import time
from typing import List, Dict, Any, Optional
from datetime import datetime

from .config import Config

class MITRECWEAPIClient:
    """MITRE CWE API 클라이언트"""
    
    def __init__(self):
        self.mitre_base_url = Config.MITRE_CWE_BASE_URL
        self.download_url = Config.MITRE_CWE_DOWNLOAD_URL
        self.cache_dir = "data/cwe_cache"
        self.cwe_data_file = os.path.join(self.cache_dir, "cwe_data.xml")
        
        # 다운로드된 ZIP 파일 경로
        self.local_zip_file = "cwec_latest.xml.zip"
        
        # 캐시 디렉토리 생성
        os.makedirs(self.cache_dir, exist_ok=True)
        
        # 설정 검증
        Config.validate_config()
    
    def download_cwe_data(self) -> bool:
        """MITRE CWE 데이터 다운로드 또는 로컬 파일 사용"""
        try:
            # 먼저 로컬 ZIP 파일 확인
            if os.path.exists(self.local_zip_file):
                print("로컬 ZIP 파일을 사용합니다...")
                return self._extract_from_local_zip()
            
            print("MITRE CWE 데이터 다운로드 중...")
            
            # ZIP 파일 다운로드
            response = requests.get(self.download_url)
            if response.status_code != 200:
                print(f"CWE 데이터 다운로드 실패: {response.status_code}")
                return False
            
            # ZIP 파일 압축 해제
            with zipfile.ZipFile(io.BytesIO(response.content)) as zip_file:
                return self._extract_xml_from_zip(zip_file)
                
        except Exception as e:
            print(f"CWE 데이터 다운로드 중 오류: {e}")
            return False
    
    def _extract_from_local_zip(self) -> bool:
        """로컬 ZIP 파일에서 XML 추출"""
        try:
            with zipfile.ZipFile(self.local_zip_file, 'r') as zip_file:
                return self._extract_xml_from_zip(zip_file)
        except Exception as e:
            print(f"로컬 ZIP 파일 처리 중 오류: {e}")
            return False
    
    def _extract_xml_from_zip(self, zip_file) -> bool:
        """ZIP 파일에서 XML 추출"""
        # XML 파일 찾기
        xml_files = [f for f in zip_file.namelist() if f.endswith('.xml')]
        if not xml_files:
            print("ZIP 파일에서 XML 파일을 찾을 수 없습니다.")
            return False
        
        # 첫 번째 XML 파일 추출
        xml_content = zip_file.read(xml_files[0])
        
        # XML 파일 저장
        with open(self.cwe_data_file, 'wb') as f:
            f.write(xml_content)
        
        print(f"CWE 데이터 추출 완료: {xml_files[0]}")
        return True
    
    def load_cwe_data(self) -> Optional[ET.Element]:
        """CWE 데이터 로드"""
        if not os.path.exists(self.cwe_data_file):
            print("CWE 데이터 파일이 없습니다. 다운로드를 시도합니다...")
            if not self.download_cwe_data():
                return None
        
        try:
            print(f"XML 파일 경로: {self.cwe_data_file}")
            print(f"XML 파일 크기: {os.path.getsize(self.cwe_data_file)} bytes")
            
            tree = ET.parse(self.cwe_data_file)
            root = tree.getroot()
            print("CWE 데이터 로드 완료")
            print(f"루트 태그: {root.tag}")
            
            # Weakness 요소 수 확인
            ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
            weaknesses = root.findall('.//cwe:Weakness', ns)
            print(f"발견된 Weakness 요소 수: {len(weaknesses)}")
            
            if weaknesses:
                print(f"첫 번째 Weakness ID: {weaknesses[0].get('ID')}")
            
            return root
        except Exception as e:
            print(f"CWE 데이터 로드 중 오류: {e}")
            return None
    
    def search_cwe_by_id(self, cwe_id: str) -> Optional[Dict[str, Any]]:
        """특정 CWE ID로 검색"""
        print(f"CWE {cwe_id} 검색 시작...")
        root = self.load_cwe_data()
        if not root:
            print("CWE 데이터를 로드할 수 없습니다.")
            return None
        
        try:
            # CWE ID에서 "CWE-" 접두사 제거 (예: "CWE-119" -> "119")
            if cwe_id.startswith('CWE-'):
                search_id = cwe_id[4:]  # "CWE-" 제거
            else:
                search_id = cwe_id
            
            print(f"검색할 ID: {search_id}")
            
            # XML 네임스페이스 정의
            ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
            
            print(f"네임스페이스: {ns}")
            print(f"루트 태그: {root.tag}")
            
            # 네임스페이스를 고려한 Weakness 요소 검색
            weaknesses = root.findall('.//cwe:Weakness', ns)
            print(f"검색된 Weakness 요소 수: {len(weaknesses)}")
            
            for i, weakness in enumerate(weaknesses):
                weakness_id = weakness.get('ID')
                if i < 5:  # 처음 5개만 출력
                    print(f"  Weakness {i+1}: ID={weakness_id}, Name={weakness.get('Name', 'N/A')}")
                
                if weakness_id == search_id:
                    print(f"CWE {cwe_id} 발견!")
                    return self._parse_cwe_element(weakness)
            
            print(f"CWE {cwe_id}를 찾을 수 없습니다.")
            return None
            
        except Exception as e:
            print(f"CWE {cwe_id} 검색 중 오류: {e}")
            return None
    
    def _parse_cwe_element(self, weakness: ET.Element) -> Dict[str, Any]:
        """CWE XML 요소 파싱"""
        cwe_data = {
            'cweId': weakness.get('ID', 'Unknown'),
            'name': weakness.get('Name', 'No name'),
            'description': '',
            'status': weakness.get('Status', 'Unknown'),
            'abstraction': weakness.get('Abstraction', 'Unknown'),
            'structure': weakness.get('Structure', 'Unknown'),
            'mitigations': [],
            'examples': []
        }
        
        # XML 네임스페이스 정의
        ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
        
        # Description 요소 찾기 (네임스페이스 고려)
        description_elem = weakness.find('.//cwe:Description', ns)
        if description_elem is not None:
            cwe_data['description'] = description_elem.text or 'No description'
        
        # Extended Description도 추가 (네임스페이스 고려)
        extended_desc_elem = weakness.find('.//cwe:Extended_Description', ns)
        if extended_desc_elem is not None and extended_desc_elem.text:
            cwe_data['description'] += f"\n\n{extended_desc_elem.text}"
        
        # 완화 방법 추출 (네임스페이스 고려)
        for mitigation in weakness.findall('.//cwe:Mitigation', ns):
            if mitigation.text:
                cwe_data['mitigations'].append({
                    'description': mitigation.text,
                    'effectiveness': mitigation.get('Effectiveness', 'Unknown')
                })
        
        # 예시 추출 (네임스페이스 고려)
        for example in weakness.findall('.//cwe:Example', ns):
            if example.text:
                cwe_data['examples'].append({
                    'description': example.text,
                    'language': example.get('Language', 'Unknown')
                })
        
        return cwe_data
    
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
    
    def collect_all_ros_cwes(self) -> List[Dict[str, Any]]:
        """모든 ROS 관련 CWE 수집"""
        print("ROS 관련 CWE 수집 시작...")
        
        all_cwes = []
        
        # 1. 카테고리별 CWE 수집
        for category in Config.ROS_CWE_CATEGORIES.keys():
            category_cwes = self.search_cwes_by_category(category)
            all_cwes.extend(category_cwes)
        
        # 2. 컴포넌트별 CWE 수집
        for component in Config.ROS_COMPONENT_CWE_MAPPING.keys():
            component_cwes = self.search_ros_component_cwes(component)
            all_cwes.extend(component_cwes)
        
        # 3. 중복 제거
        unique_cwes = []
        seen_ids = set()
        
        for cwe in all_cwes:
            cwe_id = cwe.get('cweId', 'Unknown')
            if cwe_id not in seen_ids:
                unique_cwes.append(cwe)
                seen_ids.add(cwe_id)
        
        print(f"총 {len(unique_cwes)}개의 고유한 ROS 관련 CWE 발견!")
        return unique_cwes
    
    def test_api_connection(self):
        """API 연결 테스트"""
        print("=== MITRE CWE API 연결 테스트 ===")
        print(f"Base URL: {self.mitre_base_url}")
        print(f"Download URL: {self.download_url}")
        
        try:
            # 1. 기본 연결 테스트
            print("\n1. MITRE CWE 웹사이트 연결 테스트...")
            response = requests.get(self.mitre_base_url)
            print(f"응답 상태: {response.status_code}")
            
            if response.status_code == 200:
                print("MITRE CWE 웹사이트 연결 성공!")
            else:
                print(f"MITRE CWE 웹사이트 연결 실패: {response.status_code}")
            
            # 2. CWE 데이터 다운로드 테스트
            print("\n2. CWE 데이터 다운로드 테스트...")
            if self.download_cwe_data():
                print("CWE 데이터 다운로드 성공!")
                
                # 3. 특정 CWE 검색 테스트
                print("\n3. CWE 검색 테스트...")
                test_cwe = self.search_cwe_by_id("CWE-287")
                if test_cwe:
                    print("CWE 검색 성공!")
                    print(f"테스트 CWE: {test_cwe.get('cweId')} - {test_cwe.get('name')}")
                else:
                    print("CWE 검색 실패")
            else:
                print("CWE 데이터 다운로드 실패")
                
        except Exception as e:
            print(f"API 테스트 중 오류: {e}")
    
    def format_cwe_for_rag(self, cwe_data: Dict[str, Any]) -> str:
        """CWE 데이터를 RAG용 텍스트로 변환"""
        cwe_id = cwe_data.get('cweId', 'Unknown')
        name = cwe_data.get('name', 'No name')
        description = cwe_data.get('description', 'No description')
        
        # 완화 방법 추출
        mitigations = []
        if 'mitigations' in cwe_data:
            for mitigation in cwe_data['mitigations']:
                if 'description' in mitigation:
                    mitigations.append(mitigation['description'])
        
        # 예시 추출
        examples = []
        if 'examples' in cwe_data:
            for example in cwe_data['examples']:
                if 'description' in example:
                    examples.append(example['description'])
        
        # ROS 관련성 정보
        ros_info = ""
        if 'ros_component' in cwe_data:
            ros_info = f"**ROS 컴포넌트**: {cwe_data['ros_component']}\n"
        if 'category' in cwe_data:
            ros_info += f"**취약점 카테고리**: {cwe_data['category']}\n"
        
        return f"""# CWE: {cwe_id} - {name}

## 설명
{description}

{ros_info}
## 완화 방법
{chr(10).join(f"- {mit}" for mit in mitigations) if mitigations else "- 정보 없음"}

## 예시
{chr(10).join(f"- {ex}" for ex in examples) if examples else "- 정보 없음"}

---
"""
