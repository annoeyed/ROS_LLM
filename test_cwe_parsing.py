#!/usr/bin/env python3
"""
CWE XML 파싱 테스트 스크립트
"""

import xml.etree.ElementTree as ET

def test_cwe_parsing():
    """CWE XML 파싱 테스트"""
    try:
        # XML 파일 파싱
        tree = ET.parse('cwec_v4.17.xml')
        root = tree.getroot()
        
        print("XML 로드 성공!")
        print(f"루트 태그: {root.tag}")
        
        # XML 네임스페이스 정의
        ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
        
        # 네임스페이스를 고려한 Weakness 요소 찾기
        weaknesses_with_ns = root.findall('.//cwe:Weakness', ns)
        print(f"네임스페이스 고려한 Weakness 요소 수: {len(weaknesses_with_ns)}")
        
        # 네임스페이스 없이도 시도
        weaknesses_without_ns = root.findall('.//Weakness')
        print(f"네임스페이스 없이 Weakness 요소 수: {len(weaknesses_without_ns)}")
        
        # 모든 하위 요소 확인
        all_elements = root.findall('.//*')
        print(f"총 하위 요소 수: {len(all_elements)}")
        
        # Weakness로 시작하는 태그 찾기
        weakness_tags = [elem.tag for elem in all_elements if 'Weakness' in elem.tag]
        print(f"Weakness 관련 태그들: {set(weakness_tags)}")
        
        if weaknesses_with_ns:
            # 첫 번째 Weakness 요소 확인
            first_weakness = weaknesses_with_ns[0]
            print(f"\n첫 번째 Weakness (네임스페이스 고려):")
            print(f"  ID: {first_weakness.get('ID')}")
            print(f"  Name: {first_weakness.get('Name')}")
            print(f"  Status: {first_weakness.get('Status')}")
            
        elif weaknesses_without_ns:
            # 첫 번째 Weakness 요소 확인
            first_weakness = weaknesses_without_ns[0]
            print(f"\n첫 번째 Weakness (네임스페이스 없이):")
            print(f"  ID: {first_weakness.get('ID')}")
            print(f"  Name: {first_weakness.get('Name')}")
            print(f"  Status: {first_weakness.get('Status')}")
        
        # 특정 CWE ID 검색 테스트
        print(f"\nCWE-287 검색 테스트:")
        found_287 = False
        for weakness in weaknesses_with_ns + weaknesses_without_ns:
            if weakness.get('ID') == '287':
                print(f"  CWE-287 발견: {weakness.get('Name')}")
                found_287 = True
                break
        
        if not found_287:
            print("  CWE-287을 찾을 수 없음")
                
        # CWE-1004 검색 테스트 (XML에서 확인된 것)
        print(f"\nCWE-1004 검색 테스트:")
        found_1004 = False
        for weakness in weaknesses_with_ns + weaknesses_without_ns:
            if weakness.get('ID') == '1004':
                print(f"  CWE-1004 발견: {weakness.get('Name')}")
                found_1004 = True
                break
        
        if not found_1004:
            print("  CWE-1004을 찾을 수 없음")
        
    except Exception as e:
        print(f"오류 발생: {e}")

if __name__ == "__main__":
    test_cwe_parsing()
