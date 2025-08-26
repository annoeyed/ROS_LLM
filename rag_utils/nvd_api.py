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
        
        # API key validation
        if not api_key:
            print(" NVD API key not available. Limited requests only.")
        else:
            print(f"NVD API key set: {api_key[:10]}...")
    
    def search_ros_cves(self, days_back: int = 365) -> List[Dict[str, Any]]:
        """Search for ROS-related CVEs - using NVD API v2.0"""
        # ROS-related search queries
        ros_search_queries = [
            "ROS", "Gazebo", "MoveIt", "RViz", "rclpy", "rclcpp", 
            "ament", "tf2", "urdf", "rosbag"
        ]
        
        all_cves = []
        
        for query in ros_search_queries:
            # Exact search parameters for NVD API v2.0
            params = {
                "keywordSearch": query,  # Use "keywordSearch" in v2.0
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
                    print(f" API key authentication failed: {response.status_code}")
                    print("Please obtain a valid NVD API key: https://nvd.nist.gov/developers/request-an-api-key")
                    break
                else:
                    print(f"API error for '{query}': {response.status_code}")
                    print(f"Response: {response.text[:300]}")
                    
                # Prevent API call rate limiting
                import time
                time.sleep(1)
                
            except Exception as e:
                print(f"Error searching for '{query}': {e}")
        
        return all_cves
    
    def search_ros_cves_by_cpe(self) -> List[Dict[str, Any]]:
        """Search for ROS-related product CVEs via CPE - modified version"""
        # Use simpler CPE patterns
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
                
                # Prevent API call rate limiting
                import time
                time.sleep(1)
                
            except Exception as e:
                print(f"CPE search error: {e}")
        
        return all_cves
    
    def search_ros_cves_simple(self) -> List[Dict[str, Any]]:
        """Most basic search method - for testing"""
        # Most basic search
        simple_queries = ["ROS"]
        
        all_cves = []
        
        for query in simple_queries:
            # Basic search parameters
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
                    
                # Prevent API call rate limiting
                import time
                time.sleep(1)
                
            except Exception as e:
                print(f"Simple search error: {e}")
        
        return all_cves
    
    def test_api_connection(self):
        """Test API connection"""
        print("=== NVD API Connection Test ===")
        print(f"Base URL: {self.base_url}")
        print(f"Headers: {self.headers}")
        
        try:
            # 1. Basic endpoint test
            print("\n1. Testing basic endpoint...")
            response = requests.get(self.base_url, headers=self.headers)
            print(f"Basic response status: {response.status_code}")
            print(f"Basic response URL: {response.url}")
            
            if response.status_code == 200:
                print("Basic endpoint connection successful!")
            else:
                print(f"Basic endpoint connection failed: {response.status_code}")
                print(f"Error response: {response.text[:500]}")
            
            # 2. Search parameter test
            print("\n2. Testing search parameters...")
            params = {"resultsPerPage": 1}
            response = requests.get(self.base_url, headers=self.headers, params=params)
            print(f"Search parameter response status: {response.status_code}")
            print(f"Search parameter response URL: {response.url}")
            
            if response.status_code == 200:
                data = response.json()
                print(f"Search parameter test successful!")
                print(f"Response keys: {list(data.keys())}")
                if 'vulnerabilities' in data:
                    print(f"CVE count: {len(data['vulnerabilities'])}")
            else:
                print(f"Search parameter test failed: {response.status_code}")
                print(f"Error response: {response.text[:500]}")
            
            # 3. Test other endpoints
            print("\n3. Testing other endpoints...")
            test_urls = [
                "https://services.nvd.nist.gov/rest/json/cves/2.0",
                "https://services.nvd.nist.gov/rest/json/cves/2.0/",
                "https://services.nvd.nist.gov/rest/json/cves/1.1",
                "https://services.nvd.nist.gov/rest/json/cves/1.0"
            ]
            
            for test_url in test_urls:
                try:
                    print(f"Test URL: {test_url}")
                    response = requests.get(test_url, headers=self.headers)
                    print(f"  Response status: {response.status_code}")
                    if response.status_code == 200:
                        print(f"  Success! Use this URL: {test_url}")
                        break
                except Exception as e:
                                            print(f"  Error: {e}")
                
        except Exception as e:
            print(f"Error during API test: {e}")
            print(f"Error type: {type(e).__name__}")
    
    def format_cve_for_rag(self, cve_data: Dict[str, Any]) -> str:
        """Convert CVE data to RAG text format"""
        cve_id = cve_data.get('cve', {}).get('id', 'Unknown')
        description = cve_data.get('cve', {}).get('descriptions', [{}])[0].get('value', 'No description')
        severity = cve_data.get('cve', {}).get('metrics', {}).get('cvssMetricV31', [{}])[0].get('cvssData', {}).get('baseSeverity', 'Unknown')
        
        return f"""# CVE: {cve_id}
Severity: {severity}
Description: {description}
"""