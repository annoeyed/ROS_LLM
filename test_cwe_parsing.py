#!/usr/bin/env python3
"""
CWE XML Parsing Test Script
"""

import xml.etree.ElementTree as ET

def test_cwe_parsing():
    """CWE XML Parsing Test"""
    try:
        # Parse XML file
        tree = ET.parse('cwec_v4.17.xml')
        root = tree.getroot()
        
        print("XML loaded successfully!")
        print(f"Root tag: {root.tag}")
        
        # Define XML namespace
        ns = {'cwe': 'http://cwe.mitre.org/cwe-7'}
        
        # Find Weakness elements considering namespace
        weaknesses_with_ns = root.findall('.//cwe:Weakness', ns)
        print(f"Number of Weakness elements with namespace: {len(weaknesses_with_ns)}")
        
        # Try without namespace as well
        weaknesses_without_ns = root.findall('.//Weakness')
        print(f"Number of Weakness elements without namespace: {len(weaknesses_without_ns)}")
        
        # Check all child elements
        all_elements = root.findall('.//*')
        print(f"Total number of child elements: {len(all_elements)}")
        
        # Find tags starting with Weakness
        weakness_tags = [elem.tag for elem in all_elements if 'Weakness' in elem.tag]
        print(f"Weakness related tags: {set(weakness_tags)}")
        
        if weaknesses_with_ns:
            # Check first Weakness element
            first_weakness = weaknesses_with_ns[0]
            print(f"\nFirst Weakness (with namespace):")
            print(f"  ID: {first_weakness.get('ID')}")
            print(f"  Name: {first_weakness.get('Name')}")
            print(f"  Status: {first_weakness.get('Status')}")
            
        elif weaknesses_without_ns:
            # Check first Weakness element
            first_weakness = weaknesses_without_ns[0]
            print(f"\nFirst Weakness (without namespace):")
            print(f"  ID: {first_weakness.get('ID')}")
            print(f"  Name: {first_weakness.get('Name')}")
            print(f"  Status: {first_weakness.get('Status')}")
        
        # Test searching for specific CWE ID
        print(f"\nCWE-287 search test:")
        found_287 = False
        for weakness in weaknesses_with_ns + weaknesses_without_ns:
            if weakness.get('ID') == '287':
                print(f"  CWE-287 found: {weakness.get('Name')}")
                found_287 = True
                break
        
        if not found_287:
            print("  CWE-287 not found")
                
        # CWE-1004 search test (confirmed in XML)
        print(f"\nCWE-1004 search test:")
        found_1004 = False
        for weakness in weaknesses_with_ns + weaknesses_without_ns:
            if weakness.get('ID') == '1004':
                print(f"  CWE-1004 found: {weakness.get('Name')}")
                found_1004 = True
                break
        
        if not found_1004:
            print("  CWE-1004 not found")
        
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == "__main__":
    test_cwe_parsing()
