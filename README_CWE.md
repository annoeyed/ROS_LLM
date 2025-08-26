# ROS CWE 데이터베이스 시스템

## 개요
이 시스템은 NVD(National Vulnerability Database)의 CWE(Common Weakness Enumeration) API를 사용하여 ROS(Robot Operating System) 관련 취약점 유형을 자동으로 수집하고 관리합니다.

## 주요 특징

### 🎯 CWE 기반 접근
- CVE 대신 CWE를 사용하여 취약점 유형별 분류
- 체계적인 취약점 카테고리 관리
- ROS 컴포넌트별 취약점 매핑

### 🔍 스마트 검색
- ROS 관련 CWE 패턴 자동 검색
- 카테고리별 분류 (인증, 권한, 입력 검증 등)
- 컴포넌트별 매핑 (rclpy, tf2, gazebo 등)

### 📊 RAG 최적화
- RAG 시스템용 텍스트 자동 생성
- 마크다운 및 일반 텍스트 형식 지원
- 벡터 데이터베이스 구축 준비

## 시스템 구조

```
rag_utils/
├── config.py           # 환경 변수 및 설정 관리
├── cwe_api.py          # NVD CWE API 클라이언트
├── cwe_database.py     # CWE 데이터베이스 관리
└── cwe_collector.py    # CWE 수집 메인 로직

data/
├── cwe_database/       # CWE 데이터 저장
└── rag_sources/        # RAG용 데이터

build_ros_cwe_db.py     # 메인 실행 스크립트
```

## 설치 및 설정

### 1. 의존성 설치
```bash
pip install -r requirements.txt
```

### 2. 환경 변수 설정
`.env` 파일을 생성하고 다음 내용을 추가하세요:

```bash
NVD_API_KEY=your_nvd_api_key_here
NVD_API_BASE_URL=https://services.nvd.nist.gov/rest/json
NVD_CWE_ENDPOINT=/cwes/2.0
NVD_REQUEST_DELAY=1
NVD_MAX_RESULTS_PER_PAGE=20
```

**NVD API 키 발급**: https://nvd.nist.gov/developers/request-an-api-key

## 사용법

### 기본 실행
```bash
python build_ros_cwe_db.py
```

### 명령행 옵션
```bash
# 새로 구축
python -m rag_utils.cwe_collector --action build --backup

# 업데이트
python -m rag_utils.cwe_collector --action update

# 통계 보기
python -m rag_utils.cwe_collector --action stats
```

## CWE 카테고리

### 🔐 인증 및 권한
- **Authentication**: CWE-287, CWE-288, CWE-290, CWE-295
- **Authorization**: CWE-285, CWE-862, CWE-863, CWE-2000

### 입력 검증 및 보안
- **Input Validation**: CWE-20, CWE-78, CWE-79, CWE-89, CWE-125
- **Memory Management**: CWE-119, CWE-787, CWE-125, CWE-190, CWE-191

### 🔄 직렬화 및 설정
- **Serialization**: CWE-502, CWE-611, CWE-776, CWE-400
- **Configuration**: CWE-16, CWE-434, CWE-494, CWE-732

### ⚡ 실시간 시스템
- **Race Conditions**: CWE-362, CWE-367, CWE-1200, CWE-835
- **Error Handling**: CWE-390, CWE-754, CWE-755, CWE-209

## ROS 컴포넌트별 취약점

| 컴포넌트 | 주요 취약점 유형 |
|----------|------------------|
| rclpy/rclcpp | 인증, 정보 노출, 설정, 오류 처리 |
| tf2 | 정보 노출, 설정, 경쟁 상태 |
| urdf | XML 외부 엔티티, 역직렬화, 입력 검증 |
| gazebo | 설정, 무한 루프, 오류 처리 |
| moveit | 명령 주입, 정보 노출, 설정 |
| rosbag | 정보 노출, 설정, 역직렬화 |
| navigation | 오류 처리, 무한 루프, 경쟁 상태 |
| control | 오류 처리, 무한 루프, 경쟁 상태 |

## 데이터 출력 형식

### 마크다운 형식
- `data/rag_sources/ros_cwe_summary.md`
- 카테고리별 분류 및 통계
- 상세한 CWE 설명

### RAG용 텍스트
- `data/rag_sources/ros_cwe_rag.txt`
- 벡터 데이터베이스 구축용
- 구조화된 텍스트 형식

## API 제한 및 최적화

### 요청 제한
- API 키 없음: 분당 5회 요청
- API 키 있음: 분당 1000회 요청
- 자동 지연 설정으로 제한 준수

### 성능 최적화
- 중복 CWE 자동 제거
- 배치 처리로 효율성 향상
- 에러 처리 및 재시도 로직

## 문제 해결

### API 연결 실패
1. API 키 확인
2. 네트워크 연결 상태 확인
3. NVD 서버 상태 확인

### 데이터 수집 실패
1. 환경 변수 설정 확인
2. 권한 문제 확인
3. 디스크 공간 확인

## 기여 및 개발

### 코드 구조
- 모듈화된 설계로 유지보수성 향상
- 설정과 로직 분리
- 에러 처리 및 로깅 강화

### 확장 가능성
- 새로운 CWE 카테고리 추가
- 추가 ROS 컴포넌트 지원
- 다른 보안 데이터베이스 연동

## 라이선스
이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 연락처
문의사항이나 버그 리포트는 이슈 트래커를 통해 제출해주세요.
