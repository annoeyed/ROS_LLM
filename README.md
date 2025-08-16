# ROS_LLM

ROS(로봇 운영 체제) 관련 CWE(Common Weakness Enumeration) ID를 추출하고 분석하는 도구입니다.

## 프로젝트 개요

이 프로젝트는 ROS 관련 보안 취약점을 식별하고 분석하기 위해 CWE 데이터베이스에서 ROS 관련 항목들을 추출하는 도구를 제공합니다.

## 주요 기능

- ROS 관련 CWE ID 자동 추출
- CWE 데이터베이스 파싱 및 분석
- ROS 보안 취약점 데이터베이스 구축

## 파일 구조

- `extract_cwe_ids.py`: CWE ID 추출 메인 스크립트
- `build_ros_cwe_db.py`: ROS CWE 데이터베이스 구축 스크립트
- `test_cwe_parsing.py`: CWE 파싱 테스트 스크립트
- `ros_cwe_review/`: ROS CWE 리뷰 관련 모듈
- `data/`: 데이터 파일들
- `requirements.txt`: Python 의존성 패키지

## 설치 및 사용법

1. 저장소 클론
```bash
git clone https://github.com/annoeyed/ROS_LLM.git
cd ROS_LLM
```

2. 의존성 설치
```bash
pip install -r requirements.txt
```

3. CWE ID 추출 실행
```bash
python extract_cwe_ids.py
```

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 기여하기

버그 리포트나 기능 제안은 이슈를 통해 제출해주세요. 풀 리퀘스트도 환영합니다.
