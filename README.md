# ROS_LLM

ROS(로봇 운영 체제) 관련 보안 취약점 분석을 위한 다중 에이전트 기반 AI 도구입니다.

## 프로젝트 개요

이 프로젝트는 ROS 관련 CWE(Common Weakness Enumeration) ID를 추출하고 분석하며, 다중 AI 에이전트를 통해 보안 가이드라인을 생성하고 시뮬레이션을 수행하는 종합적인 보안 분석 도구입니다.

## 주요 기능

### CWE 분석
- **ROS 관련 CWE ID 자동 추출**: CWE 데이터베이스에서 ROS 관련 취약점 식별
- **CWE 데이터베이스 구축**: 구조화된 ROS 보안 취약점 데이터베이스 생성
- **카테고리 분류**: CWE ID를 보안 영역별로 자동 분류

### 다중 AI 에이전트 시스템
- **Planner Agent**: 전체 워크플로우 계획 및 조율
- **Coder Agent**: 보안 코드 분석 및 개선 제안
- **Security Guide Agent**: ROS 보안 가이드라인 생성 (RAG 통합)
- **Simulation Agent**: 보안 시나리오 시뮬레이션 및 테스트

### 워크플로우 관리
- **순차적 워크플로우**: 단계별 보안 분석 프로세스
- **Phase 분리**: Generation과 Evaluation 단계 명확 구분
- **로깅 및 모니터링**: 전체 프로세스 추적 및 기록

## 워크플로우 구조

### Phase 1: Generation (코드 생성 단계)
```
Instruction → Planner → Security Guide (RAG 통합) → Coder → LLM as Judge
```

1. **Instruction**: 사용자 요청 입력
2. **Planner**: AI 기반 계획 수립 및 step-by-step 알고리즘 작성
3. **Security Guide (RAG 통합)**: CWE + RAG 기반 보안 가이드라인 생성 및 검증
4. **Coder**: Plan + Security Guide를 기반으로 ROS 코드 생성
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

## 프로젝트 구조

```
ROS_LLM/
├── agents/                    # AI 에이전트 모듈
│   ├── base_agent.py         # 기본 에이전트 클래스
│   ├── planner_agent.py      # 워크플로우 계획 에이전트
│   ├── coder_agent.py        # 코드 분석 에이전트
│   ├── security_guide_agent.py # 보안 가이드라인 에이전트 (RAG 통합)
│   └── simulation_agent.py   # 시뮬레이션 에이전트
├── app/                      # 애플리케이션 및 워크플로우
│   ├── sequential_workflow.py # 순차적 워크플로우 실행
│   ├── run_workflow.py       # 워크플로우 실행 스크립트
│   └── README_WORKFLOW.md    # 워크플로우 사용법
├── rag_utils/                # RAG(Retrieval-Augmented Generation) 유틸리티
│   └── ai_client.py         # AI 클라이언트 및 API 관리
├── data/                     # 데이터 및 리소스
│   ├── cwe_database/        # CWE 데이터베이스
│   ├── rag_sources/         # RAG 소스 데이터
│   └── ros_security_guidelines/ # ROS 보안 가이드라인
├── config/                   # 설정 파일
├── extract_cwe_ids.py        # CWE ID 추출 메인 스크립트
├── build_ros_cwe_db.py       # ROS CWE 데이터베이스 구축
└── test_*.py                 # 테스트 스크립트들
```

## 설치 및 설정

### 1. 저장소 클론
```bash
git clone https://github.com/annoeyed/ROS_LLM.git
cd ROS_LLM
```

### 2. 가상환경 생성 및 활성화
```bash
python -m venv .venv
source .venv/bin/activate  # Linux/Mac
# 또는
.venv\Scripts\activate     # Windows
```

### 3. 의존성 설치
```bash
pip install -r requirements.txt
```

### 4. 환경 변수 설정
`.env` 파일을 생성하고 필요한 API 키를 설정하세요:
```bash
cp .env.example .env
# .env 파일을 편집하여 API 키 설정
```

## 사용법

### 기본 CWE ID 추출
```bash
python extract_cwe_ids.py
```

### ROS CWE 데이터베이스 구축
```bash
python build_ros_cwe_db.py
```

### 다중 에이전트 워크플로우 실행
```bash
python app/run_workflow.py
```

### 개별 에이전트 테스트
```bash
python test_agents.py
```

## 설정

### AI 클라이언트 설정
`rag_utils/ai_client.py`에서 다양한 AI 서비스 제공자를 설정할 수 있습니다:
- OpenAI GPT
- Anthropic Claude
- Mock AI (테스트용)

### 워크플로우 설정
`app/` 디렉토리의 설정 파일을 통해 워크플로우 동작을 커스터마이징할 수 있습니다.

## 주요 특징

- **모듈화된 아키텍처**: 각 에이전트와 기능이 독립적으로 작동
- **확장 가능한 구조**: 새로운 에이전트나 기능을 쉽게 추가 가능
- **포괄적인 로깅**: 모든 작업 과정을 상세히 기록
- **에러 처리**: 견고한 에러 처리 및 복구 메커니즘
- **테스트 커버리지**: 각 모듈별 테스트 스크립트 제공
- **순차적 실행**: 모든 단계가 순차적으로 실행되어 의존성 문제 해결
- **명확한 피드백 경로**: 각 Oracle 실패 시 적절한 Agent로 피드백
- **RAG 통합**: Security Guide Agent에서 보안 가이드라인과 RAG 검증을 동시에 처리

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

## 기여하기

1. 이 저장소를 포크하세요
2. 새로운 기능 브랜치를 생성하세요 (`git checkout -b feature/amazing-feature`)
3. 변경사항을 커밋하세요 (`git commit -m 'Add amazing feature'`)
4. 브랜치에 푸시하세요 (`git push origin feature/amazing-feature`)
5. Pull Request를 생성하세요

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

## 지원

- **이슈 리포트**: [GitHub Issues](https://github.com/annoeyed/ROS_LLM/issues)
- **기능 제안**: [GitHub Discussions](https://github.com/annoeyed/ROS_LLM/discussions)
- **문서**: 각 모듈별 README 파일 참조

## 최신 업데이트

최근 업데이트된 주요 기능:
- 다중 AI 에이전트 시스템 구축
- 순차적 워크플로우 구현
- RAG 기반 보안 가이드라인 생성
- 포괄적인 테스트 스위트 추가
- Joiner 제거 및 Oracle 기반 검증 시스템 구현
- Generation과 Evaluation 단계 명확 분리
- LLM as Judge를 통한 코드 안전성 검증
