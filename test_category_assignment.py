#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CWE Category Assignment Test Script
"""

import sys
import os

# Add project root to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rag_utils.cwe_database import CWEDatabase

def test_category_assignment():
    """Category assignment test"""
    print("=== CWE Category Assignment Test ===")
    
    try:
        # Load CWE database
        cwe_db = CWEDatabase()
        print(f"Database loaded: {len(cwe_db.cwes)} CWEs")
        
        # Check existing statistics
        print("\n=== Existing Statistics ===")
        old_stats = cwe_db.get_statistics()
        print(f"By category: {old_stats.get('by_category', {})}")
        print(f"By component: {old_stats.get('by_component', {})}")
        
        # Execute category assignment
        print("\n=== Executing Category Assignment ===")
        updated_count = cwe_db.assign_categories_to_existing_cwes()
        
        # Check updated statistics
        print("\n=== Updated Statistics ===")
        new_stats = cwe_db.get_statistics()
        print(f"By category: {new_stats.get('by_category', {})}")
        print(f"By component: {new_stats.get('by_component', {})}")
        
        # Check sample CWEs
        print("\n=== Sample CWE Check ===")
        sample_cwes = cwe_db.cwes[:5]  # First 5
        for cwe in sample_cwes:
            print(f"CWE-{cwe['cwe_id']}: {cwe['name']}")
            print(f"  Category: {cwe.get('category', 'N/A')}")
            print(f"  ROS Component: {cwe.get('ros_component', 'N/A')}")
            print()
        
        print(f"Test completed! {updated_count} CWEs updated")
        
    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_category_assignment()
