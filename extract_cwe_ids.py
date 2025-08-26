#!/usr/bin/env python3
"""
Script to extract actual CWE ID list from XML
"""

import xml.etree.ElementTree as ET

def extract_actual_cwe_ids():
    """Extract actual CWE ID list from XML"""
    try:
        tree = ET.parse('cwec_v4.17.xml')
        root = tree.getroot()
        ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
        
        actual_ids = []
        for weakness in root.findall('.//cwe:Weakness', ns):
            cwe_id = weakness.get('ID')
            name = weakness.get('Name')
            actual_ids.append((cwe_id, name))
        
        return actual_ids
        
    except Exception as e:
        print(f"Error occurred: {e}")
        return []

def filter_ros_related_cwes(cwe_list):
    """Filter ROS-related CWEs"""
    ros_keywords = [
        'robot', 'autonomous', 'sensor', 'actuator', 'control system',
        'real-time', 'embedded', 'network', 'communication', 'authentication',
        'authorization', 'input validation', 'memory management', 'serialization',
        'file', 'cryptography', 'logging', 'error handling', 'race condition',
        'buffer', 'overflow', 'injection', 'xss', 'csrf', 'privilege'
    ]
    
    ros_related = []
    for cwe_id, name in cwe_list:
        name_lower = name.lower()
        if any(keyword in name_lower for keyword in ros_keywords):
            ros_related.append((cwe_id, name))
    
    return ros_related

def main():
    print("=== CWE ID Extraction and ROS Relevance Filtering ===")
    
    # 1. Extract actual CWE ID list
    print("\n1. Extracting CWE IDs from XML...")
    all_cwes = extract_actual_cwe_ids()
    print(f"Total {len(all_cwes)} CWEs found")
    
    if all_cwes:
        print("\nFirst 10 CWEs:")
        for i, (cwe_id, name) in enumerate(all_cwes[:10]):
            print(f"  {i+1:2d}. CWE-{cwe_id}: {name}")
        
        # 2. Filter ROS-related CWEs
        print("\n2. Filtering ROS-related CWEs...")
        ros_cwes = filter_ros_related_cwes(all_cwes)
        print(f"ROS-related CWEs: {len(ros_cwes)}")
        
        if ros_cwes:
            print("\nROS-related CWE list:")
            for i, (cwe_id, name) in enumerate(ros_cwes):
                print(f"  {i+1:3d}. CWE-{cwe_id}: {name}")
            
            # 3. Generate ID list for configuration file
            print("\n3. ID list for configuration file:")
            cwe_ids = [cwe_id for cwe_id, _ in ros_cwes]
            print("ROS_CWE_IDS = [")
            for i in range(0, len(cwe_ids), 5):
                batch = cwe_ids[i:i+5]
                formatted_batch = [f'"CWE-{id}"' for id in batch]
                print(f"    {', '.join(formatted_batch)},")
            print("]")
        
    else:
        print("Unable to extract CWE IDs.")

if __name__ == "__main__":
    main()
