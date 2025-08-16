#!/usr/bin/env python3
"""
CVE 데이터 품질 검증 도구
수집된 CVE가 실제로 ROS와 관련있는지 검증
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
        # ROS 관련 핵심 지표들
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
        
        # ROS와 관련없는 것들을 나타내는 지표들
        self.false_positive_indicators = [
            'roster', 'rose', 'rosetta', 'rosoft', 'roseonline',
            'rosenberg', 'rosenthal', 'ross', 'rossi'
        ]
    
    def validate_cve(self, cve_data: Dict[str, Any]) -> ValidationResult:
        """개별 CVE 검증 - 수정된 버전"""
        # CVE 데이터 구조 확인 및 디버깅
        print(f"Debug: CVE 데이터 키: {list(cve_data.keys())}")
        
        # CVE ID 추출 (여러 가능한 경로 시도)
        cve_id = None
        if 'cve_id' in cve_data:
            cve_id = cve_data['cve_id']
        elif 'cve' in cve_data and 'id' in cve_data['cve']:
            cve_id = cve_data['cve']['id']
        else:
            cve_id = 'Unknown'
        
        # 설명 추출 (여러 가능한 경로 시도)
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
        
        # ROS 관련 지표 확인
        ros_indicators = []
        confidence_score = 0.0
        
        # 높은 신뢰도 지표 확인
        for indicator in self.ros_indicators['high_confidence']:
            if indicator in description_lower:
                ros_indicators.append(indicator)
                confidence_score += 0.8
        
        # 중간 신뢰도 지표 확인
        for indicator in self.ros_indicators['medium_confidence']:
            if indicator in description_lower:
                ros_indicators.append(indicator)
                confidence_score += 0.6
        
        # 낮은 신뢰도 지표 확인
        for indicator in self.ros_indicators['low_confidence']:
            if indicator in description_lower:
                ros_indicators.append(indicator)
                confidence_score += 0.3
        
        # 거짓 양성 지표 확인
        false_positive_reasons = []
        for indicator in self.false_positive_indicators:
            if indicator in description_lower:
                false_positive_reasons.append(f"'{indicator}' 포함")
                confidence_score -= 0.5
        
        # 최종 신뢰도 점수 조정 (0.0 ~ 1.0)
        confidence_score = max(0.0, min(1.0, confidence_score))
        
        # ROS 관련성 판단 (신뢰도 0.5 이상)
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
        """전체 데이터베이스 검증"""
        if not os.path.exists(db_path):
            raise FileNotFoundError(f"데이터베이스 파일을 찾을 수 없습니다: {db_path}")
        
        with open(db_path, 'r', encoding='utf-8') as f:
            cves = json.load(f)
        
        print(f"총 {len(cves)}개의 CVE 검증 시작...")
        
        validation_results = []
        for cve in cves:
            result = self.validate_cve(cve)
            validation_results.append(result)
        
        # 통계 계산
        total_cves = len(validation_results)
        ros_related_count = sum(1 for r in validation_results if r.is_ros_related)
        false_positive_count = total_cves - ros_related_count
        
        # 신뢰도 점수 분포
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
        """검증 결과 보고서 생성"""
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write("# ROS CVE 데이터베이스 검증 보고서\n\n")
            
            # 통계 요약
            f.write("##  검증 통계\n\n")
            f.write(f"- **총 CVE 개수**: {statistics['total_cves']}\n")
            f.write(f"- **ROS 관련 CVE**: {statistics['ros_related_count']}\n")
            f.write(f"- **거짓 양성**: {statistics['false_positive_count']}\n")
            f.write(f"- **정확도**: {statistics['accuracy_rate']:.1%}\n\n")
            
            # 신뢰도 분포
            f.write("##  신뢰도 분포\n\n")
            f.write(f"- **높음 (≥0.7)**: {statistics['confidence_distribution']['high']}개\n")
            f.write(f"- **중간 (0.4-0.7)**: {statistics['confidence_distribution']['medium']}개\n")
            f.write(f"- **낮음 (<0.4)**: {statistics['confidence_distribution']['low']}개\n\n")
            
            # ROS 관련 CVE 목록
            f.write("##  ROS 관련 CVE 목록\n\n")
            ros_related = [r for r in validation_results if r.is_ros_related]
            for result in sorted(ros_related, key=lambda x: x.confidence_score, reverse=True):
                f.write(f"### {result.cve_id}\n")
                f.write(f"**신뢰도**: {result.confidence_score:.2f}\n")
                f.write(f"**ROS 지표**: {', '.join(result.ros_indicators)}\n")
                f.write(f"**설명**: {result.description}\n\n")
            
            # 거짓 양성 CVE 목록
            f.write("##  거짓 양성 CVE 목록\n\n")
            false_positives = [r for r in validation_results if not r.is_ros_related]
            for result in sorted(false_positives, key=lambda x: x.confidence_score, reverse=True):
                f.write(f"### {result.cve_id}\n")
                f.write(f"**신뢰도**: {result.confidence_score:.2f}\n")
                f.write(f"**거짓 양성 이유**: {', '.join(result.false_positive_reasons)}\n")
                f.write(f"**설명**: {result.description}\n\n")
        
        print(f"검증 보고서가 생성되었습니다: {output_path}")

def main():
    """메인 실행 함수"""
    validator = CVEValidator()
    
    try:
        # 데이터베이스 검증
        validation_results, statistics = validator.validate_database()
        
        # 결과 출력
        print("\n=== 검증 결과 ===")
        print(f"총 CVE: {statistics['total_cves']}")
        print(f"ROS 관련: {statistics['ros_related_count']}")
        print(f"거짓 양성: {statistics['false_positive_count']}")
        print(f"정확도: {statistics['accuracy_rate']:.1%}")
        
        # 보고서 생성
        validator.generate_validation_report(validation_results, statistics)
        
    except Exception as e:
        print(f"검증 중 오류 발생: {e}")

if __name__ == "__main__":
    main()
