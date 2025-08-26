#!/usr/bin/env python3
"""
ROS CWE Database Construction Execution Script
"""

import sys
import os

# Add project root to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rag_utils.cwe_collector import ROSCWECollector

def main():
    print("=== ROS CWE Database Construction Tool ===")
    print("This tool collects ROS-related vulnerability types (CWE)")
    print("using the NVD CWE API.")
    print()
    
    # NVD API key input (optional)
    api_key = input("Enter NVD API key (press Enter if none): ").strip()
    if not api_key:
        api_key = None
        print("Proceeding without API key (limited requests only)")
    else:
        # Save API key to .env file
        with open('.env', 'w') as f:
            f.write(f'NVD_API_KEY={api_key}\n')
        print("API key saved to .env file.")
    
    print()
    
    # Task selection
    print("Select task to perform:")
    print("1. Build new (delete existing data)")
    print("2. Update (keep existing data)")
    print("3. View statistics")
    print("4. Assign categories to existing CWEs")
    
    choice = input("Choice (1-4): ").strip()
    
    try:
        collector = ROSCWECollector()
        
        if choice == "1":
            # Check if backup is needed
            backup_choice = input("Backup existing data? (y/n): ").strip().lower()
            
            if backup_choice == 'y':
                backup_dir = collector.backup_existing_database()
                print(f"Backup completed: {backup_dir}")
                print()
            
            # Confirm construction
            proceed = input("Build database from scratch? (y/n): ").strip().lower()
            
            if proceed == 'y':
                print("\nStarting construction...")
                ros_cwes = collector.build_database()
                
                print(f"\nConstruction completed!")
                print(f"ROS-related CWEs found: {len(ros_cwes)}")
                
            else:
                print("Construction cancelled.")
                
        elif choice == "2":
            print("\nStarting update...")
            ros_cwes = collector.update_database()
            
            print(f"\nUpdate completed!")
            print(f"Updated ROS-related CWEs: {len(ros_cwes)}")
            
        elif choice == "3":
            collector.show_statistics()
            
        elif choice == "4":
            print("\nAssigning categories to existing CWEs...")
            updated_count = collector.cwe_db.assign_categories_to_existing_cwes()
            
            if updated_count > 0:
                print(f"\nCategory assignment completed: {updated_count} CWEs updated")
                print("Select option 3 to view updated statistics.")
            else:
                print("All CWEs already have categories assigned.")
            
        else:
            print("Invalid choice.")
            
    except Exception as e:
        print(f"Error occurred: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
