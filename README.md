# ROS 보안 코드 생성 시스템

ROS(Robot Operating System) 보안 코드 생성을 위한 멀티 에이전트 시스템입니다.

##  주요 기능

- **멀티 에이전트 아키텍처**: AI 기반 에이전트들이 협력하여 ROS 보안 코드 생성
- **CWE 기반 보안 가이드라인**: 176개 CWE 취약점에 대한 상세한 보안 가이드라인
- **RAG 시스템**: CWE 데이터베이스 기반 검색 및 인덱싱
- **AI 통합**: OpenAI GPT, Anthropic Claude 등 다양한 LLM 지원
- **모델 선택**: 환경 변수를 통한 LLM 모델 설정 및 커스터마이징

##  AI 에이전트 구성

### 1. Planner Agent
- 사용자 요청 분석 및 ROS 코드 생성 계획 수립
- AI 기반 요구사항 분석 및 작업 분해

### 2. Security Guide Agent  
- CWE 기반 보안 가이드라인 생성
- 보안 위험도 분석 및 코드 보안 검증

### 3. RAG Guard Agent
- RAG 시스템을 활용한 알고리즘 및 코드 보안 검증
- AI 기반 고급 보안 분석

### 4. Coder Agent
- 보안을 고려한 ROS 코드 생성 및 수정
- AI 기반 코드 리뷰 및 최적화

### 5. Simulation Agent
- 생성된 코드의 시뮬레이션 및 테스트
- AI 기반 테스트 시나리오 생성 및 결과 분석

##  설치 및 설정

### 1. 의존성 설치
```bash
pip install -r requirements.txt
```

### 2. AI 설정
`config/ai_config.env.example` 파일을 `.env`로 복사하고 설정:

```bash
# AI 클라이언트 타입
AI_CLIENT_TYPE=openai

# OpenAI 설정
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-4

# Anthropic 설정  
ANTHROPIC_API_KEY=your_anthropic_api_key_here
ANTHROPIC_MODEL=claude-3-sonnet-20240229

# AI 응답 설정
AI_MAX_TOKENS=1000
AI_TEMPERATURE=0.3
```

### 3. 환경 변수 설정
```bash
export AI_CLIENT_TYPE=openai
export OPENAI_API_KEY=your_key_here
export OPENAI_MODEL=gpt-4-turbo
```

##  사용 가능한 AI 모델

### OpenAI
- `gpt-4`: 가장 강력한 모델
- `gpt-4-turbo`: 빠르고 효율적
- `gpt-3.5-turbo`: 빠르고 경제적
- `gpt-4o`: 최신 모델
- `gpt-4o-mini`: 경제적

### Anthropic
- `claude-3-opus-20240229`: 가장 강력한 모델
- `claude-3-sonnet-20240229`: 균형잡힌 성능
- `claude-3-haiku-20240307`: 빠르고 경제적
- `claude-3.5-sonnet-20241022`: 최신 모델

##  테스트

### AI 설정 테스트
```bash
python test_ai_config.py
```

### 에이전트 테스트
```bash
python test_agents.py
```

### 멀티 에이전트 워크플로우 테스트
```bash
python test_multi_agent_workflow.py
```

##  사용 예시

### 기본 에이전트 사용
```python
from agents import PlannerAgent, SecurityGuideAgent, RAGGuardAgent

# 에이전트 초기화
planner = PlannerAgent()
security_guide = SecurityGuideAgent()
rag_guard = RAGGuardAgent()

# 보안 가이드라인 생성
guidelines = security_guide.generate_security_guidelines("ROS 2 노드")
```

### 멀티 에이전트 워크플로우
```python
from multi_agent_workflow import MultiAgentWorkflow

# 워크플로우 초기화
workflow = MultiAgentWorkflow()

# 전체 개발 워크플로우 실행
result = workflow.execute_workflow(
    workflow_type="full_development",
    user_request="보안이 강화된 ROS 2 노드를 생성해주세요"
)
```

##  프로젝트 구조

```
ROS_LLM/
├── agents/                 # AI 에이전트 모듈
│   ├── base_agent.py      # 기본 에이전트 클래스
│   ├── planner_agent.py   # 계획 수립 에이전트
│   ├── security_guide_agent.py  # 보안 가이드라인 에이전트
│   ├── rag_guard_agent.py # RAG 보안 검증 에이전트
│   ├── coder_agent.py     # 코드 생성 에이전트
│   └── simulation_agent.py # 시뮬레이션 에이전트
├── rag_utils/             # RAG 및 AI 유틸리티
│   ├── ai_client.py       # AI 클라이언트 인터페이스
│   └── config_loader.py   # 설정 로더
├── config/                 # 설정 파일
│   └── ai_config.env.example
├── multi_agent_workflow.py # 멀티 에이전트 워크플로우
├── test_agents.py         # 에이전트 테스트
├── test_ai_config.py      # AI 설정 테스트
└── README.md
```

##  보안 기능

- **입력 검증**: 사용자 입력에 대한 적절한 검증
- **에러 처리**: 안전한 에러 처리 및 로깅
- **보안 로깅**: 보안 관련 이벤트 추적
- **CWE 기반 검증**: 표준 취약점 패턴 검사
- **AI 기반 분석**: 고급 보안 위험 분석

##  기여하기

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

##  라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 `LICENSE` 파일을 참조하세요.

##  문의

프로젝트에 대한 문의사항이 있으시면 이슈를 생성해 주세요.
