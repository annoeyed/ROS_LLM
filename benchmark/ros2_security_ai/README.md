# ROS 2 보안 코드 생성 AI - "네 도메인" 체계

## 개요

ROS 2 보안 코드 생성 AI는 "네 도메인" 체계를 통해 보안성, 성능, 신뢰성을 보장하는 ROS 2 코드를 자동으로 생성합니다.

## 네 도메인 체계

### 1. 퍼블/섭 도메인 (Pub/Sub Domain)
- **목적**: 기본적인 ROS 2 토픽 기반 통신 구현
- **기준**: ROS 2 공식 튜토리얼 기반 최소 입력 세트
- **검증**: 60초 동안 "Hello World" ≥ 50개 수신, p95 간격 ≈1.0s, 손실률 0%

### 2. 벤치마크 도메인 (Benchmark Domain)
- **목적**: 코드의 성능과 신뢰성 측정 및 검증
- **도구**: Apex.AI performance_test, iRobot ros2-performance, NVIDIA ros2_benchmark
- **기준**: p95 지연 ≤ 50ms, 손실률 ≤ 0.1%

### 3. 서비스 도메인 (Service Domain)
- **목적**: ROS 2 서비스와 라이프사이클 노드 구현
- **기준**: 공식 예제 기반 설계, 예외 처리, 입력 검증
- **검증**: 전환 성공률, 평균 전환 시간, 서비스 응답 시간

### 4. 보안 도메인 (Security Domain)
- **목적**: SROS2 기반 보안 통신 구현
- **기준**: DDS-Security 표준, 키스토어/권한/거버넌스
- **검증**: 보안 오버헤드, 무권한 접근 차단, 통신 암호화

## 구현 구조

```
benchmark/ros2_security_ai/
├── domains/                   # 네 도메인별 구현
│   ├── pubsub/               # 퍼블/섭 도메인
│   ├── benchmark/            # 벤치마크 도메인
│   ├── service/              # 서비스 도메인
│   └── security/             # 보안 도메인
├── oracles/                  # 검증 오라클
│   ├── param_oracle.py       # 파라미터 검증 오라클
│   ├── safety_oracle.py      # 안전성 검증 오라클
│   └── mode_oracle.py        # 모드 검증 오라클
├── templates/                 # 코드 생성 템플릿
└── security_checklist/        # 보안 체크리스트
```

## 사용법

### 1. 기본 퍼블/섭 구현
```bash
python -m benchmark.ros2_security_ai.domains.pubsub.talker_listener
```

### 2. 성능 벤치마크
```bash
python -m benchmark.ros2_security_ai.domains.benchmark.apex_benchmark
```

### 3. 보안 테스트
```bash
python -m benchmark.ros2_security_ai.domains.security.sros2_test
```

## 보안 체크리스트

각 도메인별로 다음 항목들을 검증합니다:

- **입력 검증**: 빈 문자열·음수 금지, 범위 체크, 파라미터 기본값/타입 체크
- **예외 처리/로깅**: 모든 통신/전환 경로에 구조적 로깅, 예외 핸들러
- **QoS 준수**: 요청된 QoS(RELIABLE/KEEP_LAST/depth=10) 준수 여부
- **라이프사이클**: 비활성 상태 발행 금지, 전환 실패 시 복구
- **보안(SROS2)**: 올바른 enclave/permissions일 때만 통신, 무권한 차단 로그 확인
