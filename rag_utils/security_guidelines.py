#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 보안 가이드라인 생성기
CWE 데이터베이스를 기반으로 ROS 특화 보안 체크리스트와 가이드라인을 생성
"""

import json
import os
from typing import List, Dict, Any, Optional
from .cwe_database import CWEDatabase
from .cwe_rag import CWERAGSearch
from .config import Config

class SecurityGuidelineGenerator:
    def __init__(self, cwe_db: CWEDatabase, cwe_rag: CWERAGSearch):
        self.cwe_db = cwe_db
        self.cwe_rag = cwe_rag
        self.guidelines = {}
    
    def generate_ros_security_guidelines(self) -> Dict[str, Any]:
        """Generate complete ROS security guidelines"""
        print("=== Starting ROS Security Guidelines Generation ===")
        
        # 1. Generate category-based guidelines
        self.guidelines['categories'] = self._generate_category_guidelines()
        
        # 2. Generate ROS component-based guidelines
        self.guidelines['components'] = self._generate_component_guidelines()
        
        # 3. Generate development phase-based guidelines
        self.guidelines['development_phases'] = self._generate_development_phase_guidelines()
        
        # 4. Generate security checklist
        self.guidelines['checklist'] = self._generate_security_checklist()
        
        print("=== ROS Security Guidelines Generation Completed ===")
        return self.guidelines
    
    def _generate_category_guidelines(self) -> Dict[str, Any]:
        """Generate security guidelines by category"""
        category_guidelines = {}
        
        for category, cwe_ids in Config.ROS_CWE_CATEGORIES.items():
            print(f"Generating guidelines for category '{category}'...")
            
            category_info = {
                'name': category,
                'description': self._get_category_description(category),
                'risk_level': self._assess_category_risk(cwe_ids),
                'cwe_count': len(cwe_ids),
                'guidelines': [],
                'examples': [],
                'mitigations': []
            }
            
            # Generate detailed guidelines for each CWE
            for cwe_id in cwe_ids:
                cwe_guideline = self._generate_cwe_guideline(cwe_id)
                if cwe_guideline:
                    category_info['guidelines'].append(cwe_guideline)
            
            category_guidelines[category] = category_info
        
        return category_guidelines
    
    def _generate_component_guidelines(self) -> Dict[str, Any]:
        """Generate security guidelines by ROS component"""
        component_guidelines = {}
        
        for component, cwe_ids in Config.ROS_COMPONENT_CWE_MAPPING.items():
            print(f"Generating guidelines for component '{component}'...")
            
            component_info = {
                'name': component,
                'description': self._get_component_description(component),
                'risk_level': self._assess_component_risk(cwe_ids),
                'cwe_count': len(cwe_ids),
                'critical_vulnerabilities': [],
                'security_best_practices': [],
                'testing_recommendations': []
            }
            
            # Generate security recommendations for each CWE
            for cwe_id in cwe_ids:
                cwe_info = self._get_cwe_info(cwe_id)
                if cwe_info:
                    component_info['critical_vulnerabilities'].append({
                        'cwe_id': cwe_id,
                        'name': cwe_info.get('name', ''),
                        'description': cwe_info.get('description', ''),
                        'risk_level': self._assess_cwe_risk(cwe_info)
                    })
            
            component_guidelines[component] = component_info
        
        return component_guidelines
    
    def _generate_development_phase_guidelines(self) -> Dict[str, Any]:
        """Generate security guidelines by development phase"""
        phases = {
            'planning': {
                'name': '계획 단계',
                'description': '보안 요구사항 정의 및 위험 분석',
                'activities': [
                    '보안 요구사항 정의',
                    '위험 분석 및 평가',
                    '보안 아키텍처 설계',
                    '보안 테스트 계획 수립'
                ],
                'deliverables': [
                    '보안 요구사항 명세서',
                    '위험 분석 보고서',
                    '보안 아키텍처 문서',
                    '보안 테스트 계획서'
                ]
            },
            'design': {
                'name': '설계 단계',
                'description': '보안을 고려한 시스템 및 소프트웨어 설계',
                'activities': [
                    '보안 아키텍처 상세 설계',
                    '보안 인터페이스 설계',
                    '인증 및 권한 관리 설계',
                    '데이터 보호 메커니즘 설계'
                ],
                'deliverables': [
                    '보안 상세 설계서',
                    '보안 인터페이스 명세서',
                    '보안 테스트 케이스'
                ]
            },
            'implementation': {
                'name': '구현 단계',
                'description': '보안 코딩 표준 준수 및 보안 기능 구현',
                'activities': [
                    '보안 코딩 표준 준수',
                    '보안 기능 구현',
                    '코드 보안 검토',
                    '정적 분석 도구 활용'
                ],
                'deliverables': [
                    '보안 코드 검토 보고서',
                    '정적 분석 결과 보고서',
                    '보안 단위 테스트 결과'
                ]
            },
            'testing': {
                'name': '테스트 단계',
                'description': '보안 테스트 수행 및 취약점 검증',
                'activities': [
                    '보안 단위 테스트',
                    '보안 통합 테스트',
                    '침투 테스트',
                    '보안 성능 테스트'
                ],
                'deliverables': [
                    '보안 테스트 결과 보고서',
                    '취약점 보고서',
                    '보안 테스트 커버리지 보고서'
                ]
            },
            'deployment': {
                'name': '배포 단계',
                'description': '보안 설정 및 운영 환경 보안 점검',
                'activities': [
                    '보안 설정 검증',
                    '운영 환경 보안 점검',
                    '보안 모니터링 설정',
                    '사고 대응 계획 수립'
                ],
                'deliverables': [
                    '보안 배포 검증 보고서',
                    '보안 모니터링 계획서',
                    '사고 대응 계획서'
                ]
            }
        }
        
        return phases
    
    def _generate_security_checklist(self) -> Dict[str, Any]:
        """보안 체크리스트 생성"""
        checklist = {
            'authentication': {
                'name': '인증 보안 체크리스트',
                'items': [
                    '사용자 인증 메커니즘 구현 여부',
                    '강력한 비밀번호 정책 적용 여부',
                    '다중 인증(MFA) 적용 여부',
                    '세션 관리 보안 여부',
                    '로그아웃 기능 구현 여부'
                ]
            },
            'authorization': {
                'name': '권한 관리 체크리스트',
                'items': [
                    '역할 기반 접근 제어(RBAC) 구현 여부',
                    '최소 권한 원칙 적용 여부',
                    '권한 상승 방지 메커니즘 여부',
                    'API 접근 권한 제어 여부'
                ]
            },
            'input_validation': {
                'name': '입력 검증 체크리스트',
                'items': [
                    '사용자 입력 검증 구현 여부',
                    'SQL 인젝션 방지 여부',
                    'XSS 공격 방지 여부',
                    '파일 업로드 보안 검증 여부'
                ]
            },
            'data_protection': {
                'name': '데이터 보호 체크리스트',
                'items': [
                    '민감 데이터 암호화 여부',
                    '전송 중 데이터 보호 여부',
                    '저장 데이터 보안 여부',
                    '데이터 백업 및 복구 계획 여부'
                ]
            },
            'network_security': {
                'name': '네트워크 보안 체크리스트',
                'items': [
                    'TLS/SSL 적용 여부',
                    '방화벽 설정 여부',
                    '네트워크 세그멘테이션 여부',
                    '침입 탐지 시스템 구축 여부'
                ]
            },
            'logging_monitoring': {
                'name': '로깅 및 모니터링 체크리스트',
                'items': [
                    '보안 이벤트 로깅 여부',
                    '로그 무결성 보장 여부',
                    '실시간 모니터링 시스템 구축 여부',
                    '알림 및 경고 시스템 구축 여부'
                ]
            }
        }
        
        return checklist
    
    def _generate_cwe_guideline(self, cwe_id: str) -> Optional[Dict[str, Any]]:
        """특정 CWE에 대한 가이드라인 생성"""
        cwe_info = self._get_cwe_info(cwe_id)
        if not cwe_info:
            return None
        
        return {
            'cwe_id': cwe_id,
            'name': cwe_info.get('name', ''),
            'description': cwe_info.get('description', ''),
            'risk_level': self._assess_cwe_risk(cwe_info),
            'mitigation': self._get_cwe_mitigation(cwe_info),
            'examples': self._get_cwe_examples(cwe_info),
            'testing_approach': self._get_testing_approach(cwe_id)
        }
    
    def _get_cwe_info(self, cwe_id: str) -> Optional[Dict[str, Any]]:
        """CWE 정보 조회"""
        # CWE ID 정규화
        if not cwe_id.startswith('CWE-'):
            cwe_id = f"CWE-{cwe_id}"
        
        # CWE 데이터베이스에서 검색
        for cwe in self.cwe_db.cwes:
            if cwe['cwe_id'] == cwe_id.replace('CWE-', ''):
                return cwe
        
        return None
    
    def _assess_cwe_risk(self, cwe_info: Dict[str, Any]) -> str:
        """CWE 위험도 평가"""
        # 간단한 위험도 평가 로직
        if cwe_info.get('raw_data', {}).get('abstraction') == 'Base':
            return 'High'
        elif cwe_info.get('raw_data', {}).get('abstraction') == 'Variant':
            return 'Medium'
        else:
            return 'Low'
    
    def _assess_category_risk(self, cwe_ids: List[str]) -> str:
        """카테고리 위험도 평가"""
        high_risk_count = 0
        total_count = len(cwe_ids)
        
        if total_count == 0:
            return 'Low'
        
        for cwe_id in cwe_ids:
            cwe_info = self._get_cwe_info(cwe_id)
            if cwe_info and self._assess_cwe_risk(cwe_info) == 'High':
                high_risk_count += 1
        
        risk_ratio = high_risk_count / total_count
        
        if risk_ratio > 0.6:
            return 'High'
        elif risk_ratio > 0.3:
            return 'Medium'
        else:
            return 'Low'
    
    def _assess_component_risk(self, cwe_ids: List[str]) -> str:
        """컴포넌트 위험도 평가"""
        return self._assess_category_risk(cwe_ids)
    
    def _get_category_description(self, category: str) -> str:
        """카테고리 설명 반환"""
        descriptions = {
            'authentication': '사용자 및 시스템 인증과 관련된 보안 취약점',
            'authorization': '접근 권한 및 권한 관리와 관련된 보안 취약점',
            'input_validation': '사용자 입력 검증 및 처리와 관련된 보안 취약점',
            'memory_management': '메모리 할당, 해제, 접근과 관련된 보안 취약점',
            'file_operations': '파일 시스템 작업과 관련된 보안 취약점',
            'race_conditions': '동시성 및 경쟁 상태와 관련된 보안 취약점',
            'network_security': '네트워크 통신 및 보안과 관련된 보안 취약점',
            'cryptography': '암호화 및 보안 알고리즘과 관련된 보안 취약점',
            'logging': '로깅 및 모니터링과 관련된 보안 취약점',
            'error_handling': '오류 처리 및 예외 상황과 관련된 보안 취약점'
        }
        return descriptions.get(category, '보안 취약점 카테고리')
    
    def _get_component_description(self, component: str) -> str:
        """ROS 컴포넌트 설명 반환"""
        descriptions = {
            'rclpy/rclcpp': 'ROS 2 클라이언트 라이브러리 (Python/C++)',
            'tf2': '좌표 변환 라이브러리',
            'urdf': '로봇 모델링 및 설명 파일',
            'gazebo': '로봇 시뮬레이션 환경',
            'moveit': '로봇 모션 플래닝 및 제어',
            'rosbag': 'ROS 데이터 기록 및 재생',
            'navigation': '로봇 내비게이션 시스템',
            'control': '로봇 제어 시스템',
            'diagnostics': '시스템 진단 및 모니터링',
            'visualization': '로봇 데이터 시각화',
            'hardware_drivers': '하드웨어 드라이버 및 인터페이스',
            'communication': '통신 프로토콜 및 인터페이스'
        }
        return descriptions.get(component, 'ROS 컴포넌트')
    
    def _get_cwe_mitigation(self, cwe_info: Dict[str, Any]) -> List[str]:
        """CWE 완화 방안 추출"""
        mitigations = []
        raw_data = cwe_info.get('raw_data', {})
        
        if raw_data.get('mitigations'):
            for mitigation in raw_data['mitigations']:
                if mitigation.get('description'):
                    mitigations.append(mitigation['description'])
        
        # 기본 완화 방안 추가
        if not mitigations:
            mitigations.append("코드 리뷰 및 정적 분석 도구 활용")
            mitigations.append("보안 코딩 표준 준수")
            mitigations.append("정기적인 보안 테스트 수행")
        
        return mitigations
    
    def _get_cwe_examples(self, cwe_info: Dict[str, Any]) -> List[str]:
        """CWE 예시 추출"""
        examples = []
        raw_data = cwe_info.get('raw_data', {})
        
        if raw_data.get('examples'):
            for example in raw_data['examples']:
                if example.get('description'):
                    examples.append(example['description'])
        
        return examples
    
    def _get_testing_approach(self, cwe_id: str) -> str:
        """CWE별 테스트 접근법"""
        testing_approaches = {
            '287': '인증 메커니즘 테스트, 세션 관리 테스트',
            '285': '권한 검증 테스트, 접근 제어 테스트',
            '20': '입력 검증 테스트, 경계값 테스트',
            '119': '메모리 할당 테스트, 버퍼 오버플로우 테스트',
            '362': '동시성 테스트, 경쟁 상태 테스트',
            '434': '파일 업로드 테스트, 경로 검증 테스트'
        }
        
        cwe_num = cwe_id.replace('CWE-', '')
        return testing_approaches.get(cwe_num, '일반적인 보안 테스트 수행')
    
    def save_guidelines(self, output_file: str = "data/ros_security_guidelines.json"):
        """가이드라인을 JSON 파일로 저장"""
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(self.guidelines, f, ensure_ascii=False, indent=2)
        
        print(f"보안 가이드라인이 저장되었습니다: {output_file}")
    
    def generate_markdown_report(self, output_file: str = "data/ros_security_guidelines.md"):
        """가이드라인을 마크다운 보고서로 생성"""
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("# ROS 보안 가이드라인\n\n")
            f.write("이 문서는 CWE 데이터베이스를 기반으로 생성된 ROS 특화 보안 가이드라인입니다.\n\n")
            
            # 카테고리별 가이드라인
            f.write("## 1. 보안 취약점 카테고리별 가이드라인\n\n")
            for category, info in self.guidelines.get('categories', {}).items():
                f.write(f"### {info['name']}\n")
                f.write(f"**위험도**: {info['risk_level']}\n")
                f.write(f"**설명**: {info['description']}\n")
                f.write(f"**CWE 수**: {info['cwe_count']}개\n\n")
                
                for guideline in info['guidelines'][:3]:  # 상위 3개만 표시
                    f.write(f"- **CWE-{guideline['cwe_id']}**: {guideline['name']}\n")
                    f.write(f"  - 위험도: {guideline['risk_level']}\n")
                    f.write(f"  - 완화 방안: {', '.join(guideline['mitigation'][:2])}\n\n")
            
            # 컴포넌트별 가이드라인
            f.write("## 2. ROS 컴포넌트별 보안 가이드라인\n\n")
            for component, info in self.guidelines.get('components', {}).items():
                f.write(f"### {info['name']}\n")
                f.write(f"**위험도**: {info['risk_level']}\n")
                f.write(f"**설명**: {info['description']}\n")
                f.write(f"**CWE 수**: {info['cwe_count']}개\n\n")
            
            # 개발 단계별 가이드라인
            f.write("## 3. 개발 단계별 보안 가이드라인\n\n")
            for phase, info in self.guidelines.get('development_phases', {}).items():
                f.write(f"### {info['name']}\n")
                f.write(f"**설명**: {info['description']}\n\n")
                f.write("**주요 활동**:\n")
                for activity in info['activities']:
                    f.write(f"- {activity}\n")
                f.write("\n")
            
            # 보안 체크리스트
            f.write("## 4. 보안 체크리스트\n\n")
            for category, info in self.guidelines.get('checklist', {}).items():
                f.write(f"### {info['name']}\n")
                for item in info['items']:
                    f.write(f"- [ ] {item}\n")
                f.write("\n")
        
        print(f"마크다운 보고서가 생성되었습니다: {output_file}")

def test_security_guidelines():
    """보안 가이드라인 생성 테스트"""
    print("=== 보안 가이드라인 생성 테스트 시작 ===")
    
    try:
        # 1. CWE 데이터베이스 로드
        from .cwe_database import CWEDatabase
        cwe_db = CWEDatabase()
        cwe_db.load_database()
        print(f"CWE 데이터베이스 로드 완료: {len(cwe_db.cwes)}개 CWE")
        
        # 2. CWE RAG 시스템 로드
        from .cwe_rag import CWERAGSearch
        cwe_rag = CWERAGSearch()
        print("CWE RAG 시스템 로드 완료")
        
        # 3. 보안 가이드라인 생성
        generator = SecurityGuidelineGenerator(cwe_db, cwe_rag)
        guidelines = generator.generate_ros_security_guidelines()
        
        # 4. 결과 저장
        generator.save_guidelines()
        generator.generate_markdown_report()
        
        print("=== 보안 가이드라인 생성 테스트 완료 ===")
        
    except Exception as e:
        print(f"보안 가이드라인 생성 실패: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_security_guidelines()
