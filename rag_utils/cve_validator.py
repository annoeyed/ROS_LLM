#!/usr/bin/env python3
"""
CVE Data Quality Validation Tool
Validates whether collected CVEs are actually related to ROS
"""

import json
import os
from typing import List, Dict, Any, Tuple
from dataclasses import dataclass

@dataclass
class ValidationResult:
    cve_id: str
    is_ros_related: bool
    confidence_score: float
    ros_indicators: List[str]
    false_positive_reasons: List[str]
    description: str

class CVEValidator:
    def __init__(self):
        # Core indicators related to ROS
        self.ros_indicators = {
            'high_confidence': [
                'robot operating system',
                'ros 1', 'ros 2', 'ros2',
                'open robotics',
                'ros foundation'
            ],
            'medium_confidence': [
                'gazebo simulator', 'rviz', 'rqt', 'moveit',
                'urdf', 'tf2', 'rclpy', 'rclcpp', 'ament',
                'ros industrial', 'ros-industrial'
            ],
            'low_confidence': [
                'dds', 'rosbag', 'rostopic', 'tf', 'urdf'
            ]
        }
        
        # Indicators that suggest no relation to ROS
        self.false_positive_indicators = [
            'roster', 'rose', 'rosetta', 'rosoft', 'roseonline',
            'rosenberg', 'rosenthal', 'ross', 'rossi'
        ]
    
    def validate_cve(self, cve_data: Dict[str, Any]) -> ValidationResult:
        """Validate individual CVE - modified version"""
        # Check CVE data structure and debug
        print(f"Debug: CVE data keys: {list(cve_data.keys())}")
        
        # Extract CVE ID (try multiple possible paths)
        cve_id = None
        if 'cve_id' in cve_data:
            cve_id = cve_data['cve_id']
        elif 'cve' in cve_data and 'id' in cve_data['cve']:
            cve_id = cve_data['cve']['id']
        else:
            cve_id = 'Unknown'
        
        # Extract description (try multiple possible paths)
        description = ""
        if 'description' in cve_data:
            description = cve_data['description']
        elif 'cve' in cve_data and 'descriptions' in cve_data['cve']:
            descriptions = cve_data['cve']['descriptions']
            if descriptions and len(descriptions) > 0:
                description = descriptions[0].get('value', '')
        
        description_lower = description.lower()
        
        print(f"Debug: CVE ID: {cve_id}")
        print(f"Debug: Description: {description[:100]}...")
        
        # Check ROS-related indicators
        ros_indicators = []
        confidence_score = 0.0
        
        # Check high confidence indicators
        for indicator in self.ros_indicators['high_confidence']:
            if indicator in description_lower:
                ros_indicators.append(indicator)
                confidence_score += 0.8
        
        # Check medium confidence indicators
        for indicator in self.ros_indicators['medium_confidence']:
            if indicator in description_lower:
                ros_indicators.append(indicator)
                confidence_score += 0.6
        
        # Check low confidence indicators
        for indicator in self.ros_indicators['low_confidence']:
            if indicator in description_lower:
                ros_indicators.append(indicator)
                confidence_score += 0.3
        
        # Check false positive indicators
        false_positive_reasons = []
        for indicator in self.false_positive_indicators:
            if indicator in description_lower:
                false_positive_reasons.append(f"'{indicator}' included")
                confidence_score -= 0.5
        
        # Adjust final confidence score (0.0 ~ 1.0)
        confidence_score = max(0.0, min(1.0, confidence_score))
        
        # Determine ROS relevance (confidence >= 0.5)
        is_ros_related = confidence_score >= 0.5
        
        print(f"Debug: Confidence Score: {confidence_score}")
        print(f"Debug: ROS Related: {is_ros_related}")
        
        return ValidationResult(
            cve_id=cve_id,
            is_ros_related=is_ros_related,
            confidence_score=confidence_score,
            ros_indicators=ros_indicators,
            false_positive_reasons=false_positive_reasons,
            description=description[:200] + "..." if len(description) > 200 else description
        )
    
    def validate_database(self, db_path: str = "data/cve_database/cves.json") -> Tuple[List[ValidationResult], Dict[str, Any]]:
        """Validate entire database"""
        if not os.path.exists(db_path):
            raise FileNotFoundError(f"Database file not found: {db_path}")
        
        with open(db_path, 'r', encoding='utf-8') as f:
            cves = json.load(f)
        
        print(f"Starting validation of {len(cves)} CVEs...")
        
        validation_results = []
        for cve in cves:
            result = self.validate_cve(cve)
            validation_results.append(result)
        
        # Calculate statistics
        total_cves = len(validation_results)
        ros_related_count = sum(1 for r in validation_results if r.is_ros_related)
        false_positive_count = total_cves - ros_related_count
        
        # Confidence score distribution
        confidence_distribution = {
            'high': sum(1 for r in validation_results if r.confidence_score >= 0.7),
            'medium': sum(1 for r in validation_results if 0.4 <= r.confidence_score < 0.7),
            'low': sum(1 for r in validation_results if r.confidence_score < 0.4)
        }
        
        statistics = {
            'total_cves': total_cves,
            'ros_related_count': ros_related_count,
            'false_positive_count': false_positive_count,
            'accuracy_rate': ros_related_count / total_cves if total_cves > 0 else 0,
            'confidence_distribution': confidence_distribution
        }
        
        return validation_results, statistics
    
    def generate_validation_report(self, validation_results: List[ValidationResult], 
                                 statistics: Dict[str, Any], 
                                 output_path: str = "data/cve_validation_report.md"):
        """Generate validation result report"""
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write("# ROS CVE Database Validation Report\n\n")
            
            # Statistics summary
            f.write("## Validation Statistics\n\n")
            f.write(f"- **Total CVE count**: {statistics['total_cves']}\n")
            f.write(f"- **ROS-related CVEs**: {statistics['ros_related_count']}\n")
            f.write(f"- **False positives**: {statistics['false_positive_count']}\n")
            f.write(f"- **Accuracy**: {statistics['accuracy_rate']:.1%}\n\n")
            
            # Confidence distribution
            f.write("## Confidence Distribution\n\n")
            f.write(f"- **High (≥0.7)**: {statistics['confidence_distribution']['high']}\n")
            f.write(f"- **Medium (0.4-0.7)**: {statistics['confidence_distribution']['medium']}\n")
            f.write(f"- **Low (<0.4)**: {statistics['confidence_distribution']['low']}\n\n")
            
            # ROS-related CVE list
            f.write("## ROS-related CVE List\n\n")
            ros_related = [r for r in validation_results if r.is_ros_related]
            for result in sorted(ros_related, key=lambda x: x.confidence_score, reverse=True):
                f.write(f"### {result.cve_id}\n")
                f.write(f"**Confidence**: {result.confidence_score:.2f}\n")
                f.write(f"**ROS Indicators**: {', '.join(result.ros_indicators)}\n")
                f.write(f"**Description**: {result.description}\n\n")
            
            # False positive CVE list
            f.write("## False Positive CVE List\n\n")
            false_positives = [r for r in validation_results if not r.is_ros_related]
            for result in sorted(false_positives, key=lambda x: x.confidence_score, reverse=True):
                f.write(f"### {result.cve_id}\n")
                f.write(f"**Confidence**: {result.confidence_score:.2f}\n")
                f.write(f"**False Positive Reasons**: {', '.join(result.false_positive_reasons)}\n")
                f.write(f"**Description**: {result.description}\n\n")
        
        print(f"Validation report generated: {output_path}")

def main():
    """Main execution function"""
    validator = CVEValidator()
    
    try:
        # Validate database
        validation_results, statistics = validator.validate_database()
        
        # Output results
        print("\n=== Validation Results ===")
        print(f"Total CVEs: {statistics['total_cves']}")
        print(f"ROS-related: {statistics['ros_related_count']}")
        print(f"False positives: {statistics['false_positive_count']}")
        print(f"Accuracy: {statistics['accuracy_rate']:.1%}")
        
        # Generate report
        validator.generate_validation_report(validation_results, statistics)
        
    except Exception as e:
        print(f"Error during validation: {e}")

if __name__ == "__main__":
    main()
