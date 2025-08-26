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
    """MITRE CWE API client"""
    
    def __init__(self):
        self.mitre_base_url = Config.MITRE_CWE_BASE_URL
        self.download_url = Config.MITRE_CWE_DOWNLOAD_URL
        self.cache_dir = "data/cwe_cache"
        self.cwe_data_file = os.path.join(self.cache_dir, "cwe_data.xml")
        
        # Downloaded ZIP file path
        self.local_zip_file = "cwec_latest.xml.zip"
        
        # Create cache directory
        os.makedirs(self.cache_dir, exist_ok=True)
        
        # Validate configuration
        Config.validate_config()
    
    def download_cwe_data(self) -> bool:
        """Download MITRE CWE data or use local file"""
        try:
            # First check local ZIP file
            if os.path.exists(self.local_zip_file):
                print("Using local ZIP file...")
                return self._extract_from_local_zip()
            
            print("Downloading MITRE CWE data...")
            
            # Download ZIP file
            response = requests.get(self.download_url)
            if response.status_code != 200:
                print(f"CWE data download failed: {response.status_code}")
                return False
            
            # Extract ZIP file
            with zipfile.ZipFile(io.BytesIO(response.content)) as zip_file:
                return self._extract_xml_from_zip(zip_file)
                
        except Exception as e:
            print(f"Error downloading CWE data: {e}")
            return False
    
    def _extract_from_local_zip(self) -> bool:
        """Extract XML from local ZIP file"""
        try:
            with zipfile.ZipFile(self.local_zip_file, 'r') as zip_file:
                return self._extract_xml_from_zip(zip_file)
        except Exception as e:
            print(f"Error processing local ZIP file: {e}")
            return False
    
    def _extract_xml_from_zip(self, zip_file) -> bool:
        """Extract XML from ZIP file"""
        # Find XML files
        xml_files = [f for f in zip_file.namelist() if f.endswith('.xml')]
        if not xml_files:
            print("No XML files found in ZIP file.")
            return False
        
        # Extract first XML file
        xml_content = zip_file.read(xml_files[0])
        
        # Save XML file
        with open(self.cwe_data_file, 'wb') as f:
            f.write(xml_content)
        
        print(f"CWE data extraction completed: {xml_files[0]}")
        return True
    
    def load_cwe_data(self) -> Optional[ET.Element]:
        """Load CWE data"""
        if not os.path.exists(self.cwe_data_file):
            print("CWE data file not found. Attempting download...")
            if not self.download_cwe_data():
                return None
        
        try:
            print(f"XML file path: {self.cwe_data_file}")
            print(f"XML file size: {os.path.getsize(self.cwe_data_file)} bytes")
            
            tree = ET.parse(self.cwe_data_file)
            root = tree.getroot()
            print("CWE data loaded successfully")
            print(f"Root tag: {root.tag}")
            
            # Check number of Weakness elements
            ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
            weaknesses = root.findall('.//cwe:Weakness', ns)
            print(f"Number of Weakness elements found: {len(weaknesses)}")
            
            if weaknesses:
                print(f"First Weakness ID: {weaknesses[0].get('ID')}")
            
            return root
        except Exception as e:
            print(f"Error loading CWE data: {e}")
            return None
    
    def search_cwe_by_id(self, cwe_id: str) -> Optional[Dict[str, Any]]:
        """Search by specific CWE ID"""
        print(f"Starting search for CWE {cwe_id}...")
        root = self.load_cwe_data()
        if not root:
            print("Unable to load CWE data.")
            return None
        
        try:
            # Remove "CWE-" prefix from CWE ID (e.g., "CWE-119" -> "119")
            if cwe_id.startswith('CWE-'):
                search_id = cwe_id[4:]  # Remove "CWE-"
            else:
                search_id = cwe_id
            
            print(f"Searching for ID: {search_id}")
            
            # Define XML namespace
            ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
            
            print(f"Namespace: {ns}")
            print(f"Root tag: {root.tag}")
            
            # Search for Weakness elements considering namespace
            weaknesses = root.findall('.//cwe:Weakness', ns)
            print(f"Number of Weakness elements found: {len(weaknesses)}")
            
            for i, weakness in enumerate(weaknesses):
                weakness_id = weakness.get('ID')
                if i < 5:  # Only print first 5
                    print(f"  Weakness {i+1}: ID={weakness_id}, Name={weakness.get('Name', 'N/A')}")
                
                if weakness_id == search_id:
                    print(f"CWE {cwe_id} found!")
                    return self._parse_cwe_element(weakness)
            
            print(f"CWE {cwe_id} not found.")
            return None
            
        except Exception as e:
            print(f"Error searching for CWE {cwe_id}: {e}")
            return None
    
    def _parse_cwe_element(self, weakness: ET.Element) -> Dict[str, Any]:
        """Parse CWE XML element"""
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
        
        # Define XML namespace
        ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
        
        # Find Description element (considering namespace)
        description_elem = weakness.find('.//cwe:Description', ns)
        if description_elem is not None:
            cwe_data['description'] = description_elem.text or 'No description'
        
        # Also add Extended Description (considering namespace)
        extended_desc_elem = weakness.find('.//cwe:Extended_Description', ns)
        if extended_desc_elem is not None and extended_desc_elem.text:
            cwe_data['description'] += f"\n\n{extended_desc_elem.text}"
        
        # Extract mitigations (considering namespace)
        for mitigation in weakness.findall('.//cwe:Mitigation', ns):
            if mitigation.text:
                cwe_data['mitigations'].append({
                    'description': mitigation.text,
                    'effectiveness': mitigation.get('Effectiveness', 'Unknown')
                })
        
        # Extract examples (considering namespace)
        for example in weakness.findall('.//cwe:Example', ns):
            if example.text:
                cwe_data['examples'].append({
                    'description': example.text,
                    'language': example.get('Language', 'Unknown')
                })
        
        return cwe_data
    
    def search_cwes_by_category(self, category: str) -> List[Dict[str, Any]]:
        """Search CWEs by category"""
        cwe_ids = Config.ROS_CWE_CATEGORIES.get(category, [])
        cwes = []
        
        print(f"Searching for CWEs in {category} category...")
        
        for cwe_id in cwe_ids:
            cwe_data = self.search_cwe_by_id(cwe_id)
            if cwe_data:
                cwe_data['category'] = category
                cwes.append(cwe_data)
            
            # Request delay
            time.sleep(Config.NVD_REQUEST_DELAY)
        
        print(f"Found {len(cwes)} CWEs in {category} category")
        return cwes
    
    def search_ros_component_cwes(self, component: str) -> List[Dict[str, Any]]:
        """Search CWEs related to ROS component"""
        cwe_ids = Config.ROS_COMPONENT_CWE_MAPPING.get(component, [])
        cwes = []
        
        print(f"Searching for CWEs in {component} component...")
        
        for cwe_id in cwe_ids:
            cwe_data = self.search_cwe_by_id(cwe_id)
            if cwe_data:
                cwe_data['ros_component'] = component
                cwes.append(cwe_data)
            
            time.sleep(Config.NVD_REQUEST_DELAY)
        
        print(f"Found {len(cwes)} CWEs in {component} component")
        return cwes
    
    def collect_all_ros_cwes(self) -> List[Dict[str, Any]]:
        """Collect all ROS-related CWEs"""
        print("Starting ROS-related CWE collection...")
        
        all_cwes = []
        
        # 1. Collect CWEs by category
        for category in Config.ROS_CWE_CATEGORIES.keys():
            category_cwes = self.search_cwes_by_category(category)
            all_cwes.extend(category_cwes)
        
        # 2. Collect CWEs by component
        for component in Config.ROS_COMPONENT_CWE_MAPPING.keys():
            component_cwes = self.search_ros_component_cwes(component)
            all_cwes.extend(component_cwes)
        
        # 3. Remove duplicates
        unique_cwes = []
        seen_ids = set()
        
        for cwe in all_cwes:
            cwe_id = cwe.get('cweId', 'Unknown')
            if cwe_id not in seen_ids:
                unique_cwes.append(cwe)
                seen_ids.add(cwe_id)
        
        print(f"Found {len(unique_cwes)} unique ROS-related CWEs!")
        return unique_cwes
    
    def test_api_connection(self):
        """Test API connection"""
        print("=== MITRE CWE API Connection Test ===")
        print(f"Base URL: {self.mitre_base_url}")
        print(f"Download URL: {self.download_url}")
        
        try:
            # 1. Basic connection test
            print("\n1. Testing MITRE CWE website connection...")
            response = requests.get(self.mitre_base_url)
            print(f"Response status: {response.status_code}")
            
            if response.status_code == 200:
                print("MITRE CWE website connection successful!")
            else:
                print(f"MITRE CWE website connection failed: {response.status_code}")
            
            # 2. CWE data download test
            print("\n2. Testing CWE data download...")
            if self.download_cwe_data():
                print("CWE data download successful!")
                
                # 3. Specific CWE search test
                print("\n3. Testing CWE search...")
                test_cwe = self.search_cwe_by_id("CWE-287")
                if test_cwe:
                    print("CWE search successful!")
                    print(f"Test CWE: {test_cwe.get('cweId')} - {test_cwe.get('name')}")
                else:
                    print("CWE search failed")
            else:
                print("CWE data download failed")
                
        except Exception as e:
            print(f"Error during API test: {e}")
    
    def format_cwe_for_rag(self, cwe_data: Dict[str, Any]) -> str:
        """Convert CWE data to RAG text format"""
        cwe_id = cwe_data.get('cweId', 'Unknown')
        name = cwe_data.get('name', 'No name')
        description = cwe_data.get('description', 'No description')
        
        # Extract mitigations
        mitigations = []
        if 'mitigations' in cwe_data:
            for mitigation in cwe_data['mitigations']:
                if 'description' in mitigation:
                    mitigations.append(mitigation['description'])
        
        # Extract examples
        examples = []
        if 'examples' in cwe_data:
            for example in cwe_data['examples']:
                if 'description' in example:
                    examples.append(example['description'])
        
        # ROS relevance information
        ros_info = ""
        if 'ros_component' in cwe_data:
            ros_info = f"**ROS Component**: {cwe_data['ros_component']}\n"
        if 'category' in cwe_data:
            ros_info += f"**Vulnerability Category**: {cwe_data['category']}\n"
        
        return f"""# CWE: {cwe_id} - {name}

## Description
{description}

{ros_info}
## Mitigation
{chr(10).join(f"- {mit}" for mit in mitigations) if mitigations else "- No information"}

## Examples
{chr(10).join(f"- {ex}" for ex in examples) if examples else "- No information"}

---
"""
