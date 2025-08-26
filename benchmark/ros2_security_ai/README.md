# ROS 2 보안 코드 생성 AI - "네 도메인" 체계

## 개요

ROS 2 보안 코드 생성 AI는 "네 도메인" 체계를 통해 보안성, 성능, 신뢰성을 보장하는 ROS 2 코드를 자동으로 생성합니다.

## 네 도메인 체계

### 1. 퍼블/섭 도메인 (Pub/Sub Domain)
- **목적**: 기본적인 ROS 2 토픽 기반 통신 구현
- **기준**: ROS 2 공식 튜토리얼 기반 최소 입력 세트
- **검증**: 60초 동안 "Hello World" ≥ 50개 수신, p95 간격 ≈1.0s, 손실률 0%
- **구현**: `/chatter` 토픽, `std_msgs/msg/String`, 1Hz 발행, QoS depth=10, RELIABLE, VOLATILE

### 2. 벤치마크 도메인 (Benchmark Domain)
- **목적**: 코드의 성능과 신뢰성 측정 및 검증
- **도구**: Apex.AI performance_test, iRobot ros2-performance, NVIDIA ros2_benchmark
- **기준**: p95 지연 ≤ 50ms, 손실률 ≤ 0.1%
- **구현**: 메시지 크기 {32B, 1KB, 64KB}, 1Hz, 60초, QoS depth=10

### 3. 서비스 도메인 (Service Domain)
- **목적**: ROS 2 서비스와 라이프사이클 노드 구현
- **기준**: 공식 예제 기반 설계, 예외 처리, 입력 검증
- **검증**: 전환 성공률, 평균 전환 시간, 서비스 응답 시간
- **구현**: AddTwoInts 서비스, 음수 입력 거부, 에러 코드/메시지 반환

### 4. 보안 도메인 (Security Domain)
- **목적**: SROS2 기반 보안 통신 구현
- **기준**: DDS-Security 표준, 키스토어/권한/거버넌스
- **검증**: 보안 오버헤드, 무권한 접근 차단, 통신 암호화
- **구현**: 3단계 보안 실험 (OFF/ON/무권한)

## 구현 구조

```
benchmark/ros2_security_ai/
├── domains/                   # 네 도메인별 구현
│   ├── pubsub/               # 퍼블/섭 도메인
│   │   ├── __init__.py
│   │   └── talker_listener.py
│   ├── benchmark/            # 벤치마크 도메인
│   │   ├── __init__.py
│   │   └── apex_benchmark.py
│   ├── service/              # 서비스 도메인
│   │   ├── __init__.py
│   │   └── add_two_ints.py
│   └── security/             # 보안 도메인
│       ├── __init__.py
│       └── sros2_test.py
├── security_checklist/        # 보안 체크리스트
│   └── security_checklist.py
├── main.py                   # 메인 실행 파일
├── requirements.txt           # 의존성 패키지
└── README.md                 # 이 파일
```

## 설치 및 설정

### 1. 의존성 설치
```bash
cd benchmark/ros2_security_ai
pip install -r requirements.txt
```

### 2. ROS 2 환경 설정
```bash
source /opt/ros/humble/setup.bash
```

## 사용법

### 1. 전체 시스템 실행
```bash
python main.py
```

### 2. 개별 도메인 실행

#### 퍼블/섭 도메인
```bash
python -m domains.pubsub.talker_listener
```

#### 벤치마크 도메인
```bash
python -m domains.benchmark.apex_benchmark
```

#### 서비스 도메인
```bash
python -m domains.service.add_two_ints
```

#### 보안 도메인
```bash
python -m domains.security.sros2_test
```

#### 보안 체크리스트
```bash
python -m security_checklist.security_checklist
```

## 보안 체크리스트

각 도메인별로 다음 항목들을 검증합니다:

- **입력 검증**: 빈 문자열·음수 금지, 범위 체크, 파라미터 기본값/타입 체크
- **예외 처리/로깅**: 모든 통신/전환 경로에 구조적 로깅, 예외 핸들러
- **QoS 준수**: 요청된 QoS(RELIABLE/KEEP_LAST/depth=10) 준수 여부
- **라이프사이클**: 비활성 상태 발행 금지, 전환 실패 시 복구
- **보안(SROS2)**: 올바른 enclave/permissions일 때만 통신, 무권한 차단 로그 확인

## 예제 출력

### 퍼블/섭 도메인 결과
```
=== 성능 통계 ===
총 수신 메시지: 60
평균 간격: 1.000초
P95 간격: 1.000초
✅ 메시지 수신 기준 통과 (≥50개)
✅ P95 간격 기준 통과 (≈1.0초)
```

### 벤치마크 도메인 결과
```
=== Apex.AI Performance Test 벤치마크 리포트 ===
크기     P50(ms)    P95(ms)    손실률(%)   처리량     상태      
----------------------------------------------------------------------
32B      25.000     25.000     0.000      1.00       SUCCESS   
1KB      30.000     30.000     0.000      1.00       SUCCESS   
64KB     45.000     45.000     0.000      1.00       SUCCESS   
```

### 보안 도메인 결과
```
=== SROS2 보안 실험 리포트 ===
테스트: 보안 OFF 테스트
보안 모드: OFF
성공: ✅
P95 지연: 25.00ms
손실률: 0.00%
CPU 사용률: 5.00%
```

## 성능 기준

### 퍼블/섭 도메인
- 메시지 수신: ≥50개 (60초 동안)
- P95 간격: ≈1.0초 (±0.1초)
- 손실률: 0%

### 벤치마크 도메인
- P95 지연: ≤50ms
- 손실률: ≤0.1%
- 메시지 크기: 32B, 1KB, 64KB

### 서비스 도메인
- 성공률: ≥80%
- 평균 응답 시간: ≤0.1초

### 보안 도메인
- 접근 제어: 무권한 접근 차단 확인
- 보안 오버헤드: 지연 및 CPU 사용률 측정

## 개발 및 확장

### 새로운 도메인 추가
1. `domains/` 디렉토리에 새 도메인 폴더 생성
2. `__init__.py`에 클래스 임포트 추가
3. `main.py`에 도메인 실행 메서드 추가

### 새로운 검증 규칙 추가
1. `security_checklist/security_checklist.py`에 새 검증 메서드 추가
2. `run_all_validations()` 메서드에 새 검증 추가

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 기여

버그 리포트, 기능 요청, 풀 리퀘스트를 환영합니다.

## 연락처

프로젝트 관련 문의사항이 있으시면 이슈를 생성해 주세요.
