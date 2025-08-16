# rag_utils/nvd_api.py
import requests
import json
from typing import List, Dict, Any
from datetime import datetime, timedelta

class NVDApiClient:
    def __init__(self, api_key: str = None):
        self.api_key = api_key
        # NVD API v2.0 사용 (성공적으로 작동함)
        self.base_url = "https://services.nvd.nist.gov/rest/json/cves/2.0"
        self.headers = {"apiKey": api_key} if api_key else {}
        
        # API 키 검증
        if not api_key:
            print(" NVD API 키가 없습니다. 제한된 요청만 가능합니다.")
        else:
            print(f"NVD API 키가 설정되었습니다: {api_key[:10]}...")
    
    def search_ros_cves(self, days_back: int = 365) -> List[Dict[str, Any]]:
        """ROS 관련 CVE 검색 - NVD API v2.0 사용"""
        # ROS 관련 검색어들
        ros_search_queries = [
            "ROS", "Gazebo", "MoveIt", "RViz", "rclpy", "rclcpp", 
            "ament", "tf2", "urdf", "rosbag"
        ]
        
        all_cves = []
        
        for query in ros_search_queries:
            # NVD API v2.0의 정확한 검색 파라미터
            params = {
                "keywordSearch": query,  # v2.0에서는 "keywordSearch" 사용
                "resultsPerPage": 20
            }
            
            try:
                print(f"Searching for: {query}")
                response = requests.get(self.base_url, headers=self.headers, params=params)
                print(f"Response status: {response.status_code}")
                print(f"Request URL: {response.url}")
                
                if response.status_code == 200:
                    data = response.json()
                    cves = data.get('vulnerabilities', [])
                    all_cves.extend(cves)
                    print(f"Found {len(cves)} CVEs for '{query}'")
                elif response.status_code == 403:
                    print(f" API 키 인증 실패: {response.status_code}")
                    print("올바른 NVD API 키를 발급받아주세요: https://nvd.nist.gov/developers/request-an-api-key")
                    break
                else:
                    print(f"API error for '{query}': {response.status_code}")
                    print(f"Response: {response.text[:300]}")
                    
                # API 호출 제한 방지
                import time
                time.sleep(1)
                
            except Exception as e:
                print(f"Error searching for '{query}': {e}")
        
        return all_cves
    
    def search_ros_cves_by_cpe(self) -> List[Dict[str, Any]]:
        """CPE를 통한 ROS 관련 제품 CVE 검색 - 수정된 버전"""
        # 더 간단한 CPE 패턴 사용
        ros_cpe_patterns = [
            "cpe:2.3:a:*:ros:*:*:*:*:*:*:*",
            "cpe:2.3:a:*:gazebo:*:*:*:*:*:*:*"
        ]
        
        all_cves = []
        
        for cpe_pattern in ros_cpe_patterns:
            params = {"cpeName": cpe_pattern}
            
            try:
                print(f"Searching CPE: {cpe_pattern}")
                response = requests.get(self.base_url, headers=self.headers, params=params)
                print(f"CPE Response status: {response.status_code}")
                
                if response.status_code == 200:
                    data = response.json()
                    cves = data.get('vulnerabilities', [])
                    all_cves.extend(cves)
                    print(f"Found {len(cves)} CVEs for CPE '{cpe_pattern}'")
                else:
                    print(f"CPE search error: {response.status_code}")
                    print(f"CPE Response: {response.text[:200]}")
                
                # API 호출 제한 방지
                import time
                time.sleep(1)
                
            except Exception as e:
                print(f"CPE search error: {e}")
        
        return all_cves
    
    def search_ros_cves_simple(self) -> List[Dict[str, Any]]:
        """가장 기본적인 검색 방법 - 테스트용"""
        # 가장 기본적인 검색
        simple_queries = ["ROS"]
        
        all_cves = []
        
        for query in simple_queries:
            # 기본 검색 파라미터
            params = {"keywordSearch": query}
            
            try:
                print(f"Simple search for: {query}")
                response = requests.get(self.base_url, headers=self.headers, params=params)
                print(f"Simple Response status: {response.status_code}")
                print(f"Simple Request URL: {response.url}")
                
                if response.status_code == 200:
                    data = response.json()
                    cves = data.get('vulnerabilities', [])
                    all_cves.extend(cves)
                    print(f"Found {len(cves)} CVEs for '{query}'")
                else:
                    print(f"Simple search error for '{query}': {response.status_code}")
                    print(f"Simple Response: {response.text[:300]}")
                    
                # API 호출 제한 방지
                import time
                time.sleep(1)
                
            except Exception as e:
                print(f"Simple search error: {e}")
        
        return all_cves
    
    def test_api_connection(self):
        """API 연결 테스트"""
        print("=== NVD API 연결 테스트 ===")
        print(f"Base URL: {self.base_url}")
        print(f"Headers: {self.headers}")
        
        try:
            # 1. 기본 엔드포인트 테스트
            print("\n1. 기본 엔드포인트 테스트...")
            response = requests.get(self.base_url, headers=self.headers)
            print(f"기본 응답 상태: {response.status_code}")
            print(f"기본 응답 URL: {response.url}")
            
            if response.status_code == 200:
                print("기본 엔드포인트 연결 성공!")
            else:
                print(f"기본 엔드포인트 연결 실패: {response.status_code}")
                print(f"오류 응답: {response.text[:500]}")
            
            # 2. 검색 파라미터 테스트
            print("\n2. 검색 파라미터 테스트...")
            params = {"resultsPerPage": 1}
            response = requests.get(self.base_url, headers=self.headers, params=params)
            print(f"검색 파라미터 응답 상태: {response.status_code}")
            print(f"검색 파라미터 응답 URL: {response.url}")
            
            if response.status_code == 200:
                data = response.json()
                print(f"검색 파라미터 테스트 성공!")
                print(f"응답 키: {list(data.keys())}")
                if 'vulnerabilities' in data:
                    print(f"CVE 개수: {len(data['vulnerabilities'])}")
            else:
                print(f"검색 파라미터 테스트 실패: {response.status_code}")
                print(f"오류 응답: {response.text[:500]}")
            
            # 3. 다른 엔드포인트 테스트
            print("\n3. 다른 엔드포인트 테스트...")
            test_urls = [
                "https://services.nvd.nist.gov/rest/json/cves/2.0",
                "https://services.nvd.nist.gov/rest/json/cves/2.0/",
                "https://services.nvd.nist.gov/rest/json/cves/1.1",
                "https://services.nvd.nist.gov/rest/json/cves/1.0"
            ]
            
            for test_url in test_urls:
                try:
                    print(f"테스트 URL: {test_url}")
                    response = requests.get(test_url, headers=self.headers)
                    print(f"  응답 상태: {response.status_code}")
                    if response.status_code == 200:
                        print(f"  성공! 이 URL을 사용하세요: {test_url}")
                        break
                except Exception as e:
                                            print(f"  오류: {e}")
                
        except Exception as e:
            print(f"API 테스트 중 오류: {e}")
            print(f"오류 타입: {type(e).__name__}")
    
    def format_cve_for_rag(self, cve_data: Dict[str, Any]) -> str:
        """CVE 데이터를 RAG용 텍스트로 변환"""
        cve_id = cve_data.get('cve', {}).get('id', 'Unknown')
        description = cve_data.get('cve', {}).get('descriptions', [{}])[0].get('value', 'No description')
        severity = cve_data.get('cve', {}).get('metrics', {}).get('cvssMetricV31', [{}])[0].get('cvssData', {}).get('baseSeverity', 'Unknown')
        
        return f"""# CVE: {cve_id}
Severity: {severity}
Description: {description}
"""