# Sequential Multi-Agent 워크플로우 시스템

## 개요

이 시스템은 이미지에 맞춘 **Generation**과 **Evaluation** 단계를 명확히 구분한 sequential 워크플로우입니다. 각 단계는 순차적으로 실행되며, 실패 시 적절한 피드백 루프를 통해 문제를 해결합니다.

**주요 변경사항**: RAG Guard Agent를 제거하고 Security Guide Agent에 RAG 기능을 통합하여 더 효율적인 구조로 개선했습니다.

## 워크플로우 구조

### Phase 1: Generation (코드 생성 단계)

```
Instruction → Planner → Security Guide (RAG 통합) → Coder → LLM as Judge
```

1. **Instruction**: 사용자 요청 입력
2. **Planner**: AI 기반 계획 수립 및 step-by-step 알고리즘 작성
3. **Security Guide (RAG 통합)**: CWE + RAG 기반 보안 가이드라인 생성 및 검증
4. **Coder**: Plan + Security Guide를 기반으로 ROS/PX4/MAVSDK 코드 생성
5. **LLM as Judge**: 생성된 코드의 안전성 검증

### Phase 2: Evaluation (검증 단계)

```
Simulation → Oracles (Param, Safety, Mode)
```

1. **Simulation**: SITL 또는 실제 하드웨어 시뮬레이션 실행
2. **Oracles**: 병렬 검증 수행
   - **Param Oracle**: 속도, 고도, 각도 등 파라미터 안전 범위 검증
   - **Safety Oracle**: 비상 절차, 금지 API, 정책 준수 검증
   - **Mode Oracle**: task specification의 mode sequence 검증

## 피드백 루프 구조

### 1. LLM Judge 실패 시
- **대상**: Coder
- **동작**: 피드백을 받아 코드 수정 후 다시 Judge 실행

### 2. Param Oracle 실패 시
- **대상**: Coder
- **동작**: 파라미터 관련 피드백을 받아 코드 수정 후 다시 Evaluation 실행

### 3. Safety Oracle 실패 시
- **대상**: Security Guide (RAG 통합)
- **동작**: 안전성 관련 피드백을 받아 가이드라인 수정 후 Coder부터 다시 실행

### 4. Mode Oracle 실패 시
- **대상**: Planner
- **동작**: 모드 시퀀스 관련 피드백을 받아 계획 수정 후 Security Guide부터 다시 실행

## 주요 특징

### Sequential 실행
- 모든 단계가 순차적으로 실행됨
- 병렬 실행 없이 명확한 의존성 관리

### 명확한 피드백 경로
- 각 Oracle 실패 시 적절한 Agent로 피드백
- 무한 루프 방지를 위한 최대 재시도 횟수 제한

### Phase 분리
- **Generation**: 코드 생성에 집중
- **Evaluation**: 검증과 피드백에 집중

### RAG 통합
- **Security Guide Agent**: CWE 데이터베이스 + RAG 시스템 통합
- **효율성 향상**: 별도 RAG Guard Agent 없이 통합된 보안 검증
- **강화된 보안**: RAG 기반 실시간 보안 위험 탐지 및 권장사항 생성

## 파일 구조

```
app/
├── sequential_workflow.py    # 메인 워크플로우 클래스 (RAG Guard Agent 제거)
├── run_workflow.py          # 워크플로우 테스트 실행 파일
└── README_WORKFLOW.md       # 이 파일
```

## 사용법

### 1. 워크플로우 실행

```python
from app.sequential_workflow import SequentialWorkflow

# 워크플로우 초기화
workflow = SequentialWorkflow()
workflow.initialize_agents()

# 워크플로우 실행
result = workflow.execute_workflow("사용자 요청")
```

### 2. 테스트 실행

```bash
cd app
python run_workflow.py
```

## 에이전트 역할

### Planner Agent
- 사용자 요청을 분석하여 step-by-step 알고리즘 작성
- 기술적 세부사항 및 보안 요구사항 도출

### Security Guide Agent (RAG 통합)
- **CWE 기반**: 정적 보안 가이드라인 생성
- **RAG 기반**: 실시간 보안 위험 탐지 및 검증
- **통합 기능**: 보안 가이드라인 + RAG 검증을 하나의 Agent에서 처리
- **동적 권장사항**: 상황에 맞는 맞춤형 보안 권장사항 생성

### Coder Agent
- 보안 가이드라인을 반영한 ROS 코드 생성
- 초기 오류 처리 및 안전 기본값 포함

### Simulation Agent
- 코드 시뮬레이션 및 종합 테스트 실행
- 보안, 성능, 기능성 검증

## 피드백 루프 예시

### 시나리오 1: Param Oracle 실패
```
Simulation → Param Oracle (FAIL) → Coder (코드 수정) → Simulation → Param Oracle (PASS)
```

### 시나리오 2: Safety Oracle 실패
```
Simulation → Safety Oracle (FAIL) → Security Guide (RAG 통합, 가이드라인 수정) → Coder → Simulation → Safety Oracle (PASS)
```

### 시나리오 3: Mode Oracle 실패
```
Simulation → Mode Oracle (FAIL) → Planner (계획 수정) → Security Guide (RAG 통합) → Coder → Simulation → Mode Oracle (PASS)
```

## 장점

1. **명확한 구조**: Generation과 Evaluation 단계가 명확히 분리
2. **적절한 피드백**: 각 Oracle 실패 시 적절한 Agent로 피드백
3. **순차적 실행**: 의존성 문제 없이 안정적인 실행
4. **확장 가능**: 새로운 Oracle이나 Agent 추가 용이
5. **효율성 향상**: RAG Guard Agent 제거로 구조 단순화
6. **통합된 보안**: Security Guide Agent에서 보안 가이드라인과 RAG 검증을 동시에 처리

## 향후 개선 사항

1. **LLM API 연동**: 실제 LLM API를 사용한 Judge 구현
2. **성능 최적화**: 불필요한 재실행 방지
3. **모니터링**: 워크플로우 실행 상태 실시간 모니터링
4. **설정 파일**: 워크플로우 설정을 외부 파일로 관리
5. **RAG 시스템 강화**: 더 정교한 보안 위험 탐지 알고리즘
