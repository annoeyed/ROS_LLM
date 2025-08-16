#!/usr/bin/env python3
"""
XML에서 실제 CWE ID 목록 추출 스크립트
"""

import xml.etree.ElementTree as ET

def extract_actual_cwe_ids():
    """XML에서 실제 CWE ID 목록 추출"""
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
        print(f"오류 발생: {e}")
        return []

def filter_ros_related_cwes(cwe_list):
    """ROS 관련 CWE 필터링"""
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
    print("=== CWE ID 추출 및 ROS 관련성 필터링 ===")
    
    # 1. 실제 CWE ID 목록 추출
    print("\n1. XML에서 CWE ID 추출 중...")
    all_cwes = extract_actual_cwe_ids()
    print(f"총 {len(all_cwes)}개의 CWE 발견")
    
    if all_cwes:
        print("\n처음 10개 CWE:")
        for i, (cwe_id, name) in enumerate(all_cwes[:10]):
            print(f"  {i+1:2d}. CWE-{cwe_id}: {name}")
        
        # 2. ROS 관련 CWE 필터링
        print("\n2. ROS 관련 CWE 필터링 중...")
        ros_cwes = filter_ros_related_cwes(all_cwes)
        print(f"ROS 관련 CWE: {len(ros_cwes)}개")
        
        if ros_cwes:
            print("\nROS 관련 CWE 목록:")
            for i, (cwe_id, name) in enumerate(ros_cwes):
                print(f"  {i+1:3d}. CWE-{cwe_id}: {name}")
            
            # 3. 설정 파일용 ID 목록 생성
            print("\n3. 설정 파일용 ID 목록:")
            cwe_ids = [cwe_id for cwe_id, _ in ros_cwes]
            print("ROS_CWE_IDS = [")
            for i in range(0, len(cwe_ids), 5):
                batch = cwe_ids[i:i+5]
                formatted_batch = [f'"CWE-{id}"' for id in batch]
                print(f"    {', '.join(formatted_batch)},")
            print("]")
        
    else:
        print("CWE ID를 추출할 수 없습니다.")

if __name__ == "__main__":
    main()
