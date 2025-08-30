from typing import List, Optional, Literal, Any
from pydantic import BaseModel, Field

# QoS 정책 정의
Reliability = Literal["reliable", "best_effort"]
Durability = Literal["volatile", "transient_local"]
History = Literal["keep_last", "keep_all"]

class QoS(BaseModel):
    reliability: Reliability
    durability: Durability
    depth: int = 10
    history: History = "keep_last"


# 파라미터 정의
class Parameter(BaseModel):
    name: str
    type: Literal["int", "float", "string", "bool"]
    default: Any
    min: Optional[Any] = None
    max: Optional[Any] = None
    unit: Optional[str] = None
    description: Optional[str] = None


# Timer (주기성 동작)
class TimerSpec(BaseModel):
    id: str
    period_sec: float
    jitter_tolerance_sec: float = 0.0


# Publisher 정의
class PubSpec(BaseModel):
    topic: str
    msg_type: str
    qos: QoS
    rate_hz: Optional[float] = None
    latch: Optional[bool] = None
    description: Optional[str] = None


# Subscriber 정의
class SubSpec(BaseModel):
    topic: str
    msg_type: str
    qos: QoS
    queue_callback: int = 10
    description: Optional[str] = None


# 에러 처리 정책
class ErrorRetry(BaseModel):
    max_retries: int = 3
    backoff: Literal["fixed", "linear", "exponential"] = "exponential"
    base_delay_ms: int = 200

class ErrorHandling(BaseModel):
    policy: Literal["retry", "fail_fast", "circuit_breaker"] = "retry"
    retry: ErrorRetry = ErrorRetry()
    on_param_invalid: Literal["clamp", "reject_start", "warn_and_continue"] = "clamp"
    on_msg_drop: Literal["log_debug", "log_warn", "log_error", "count_metric"] = "log_warn"


# 진단/헬스체크
class Diagnostics(BaseModel):
    enable: bool = False
    heartbeat_topic: Optional[str] = None
    heartbeat_rate_hz: Optional[float] = None


# 런치 관련 (리매핑, 환경변수 등)
class LaunchRemap(BaseModel):
    from_: str = Field(alias="from")
    to: str

class LaunchSpec(BaseModel):
    remappings: List[LaunchRemap] = []
    env: dict = {}
    ros_args: List[str] = []


# Node 스펙
class NodeSpec(BaseModel):
    name: str
    namespace: str = ""
    executable: Optional[str] = None
    description: Optional[str] = None
    timers: List[TimerSpec] = []
    pubs: List[PubSpec] = []
    subs: List[SubSpec] = []
    parameters: List[Parameter] = []
    error_handling: ErrorHandling = ErrorHandling()
    diagnostics: Diagnostics = Diagnostics()
    launch: LaunchSpec = LaunchSpec()


# 패키지 스펙
class PackageSpec(BaseModel):
    name: str
    lang: Literal["rclpy", "rclcpp"]
    build_type: Literal["ament_python", "ament_cmake"]


# 실행 명세 전체 (최상위)
class ExecutionSpec(BaseModel):
    version: Literal["1.0"] = "1.0"
    package: PackageSpec
    nodes: List[NodeSpec]
    auto_apply: bool = False  # 하이브리드 정책 (단순 케이스는 자동 반영)
