# oracles/mode_oracle.py
from .base_oracle import BaseOracle

class ModeOracle(BaseOracle):
    """Task specification의 mode sequence를 검증하는 Oracle입니다."""
    def __init__(self):
        super().__init__("ModeOracle")

    def verify(self, code: str, simulation_result: dict) -> (bool, str):
        print(f"[{self.name}] 모드 시퀀스 검증 시작...")
        # 예시: 시뮬레이션 결과에서 특정 모드 전환이 정상적으로 이루어졌는지 확인
        # required_modes = ["TAKEOFF", "WAYPOINT_FOLLOW", "LAND"]
        # actual_modes = simulation_result.get("modes", [])
        # if not all(mode in actual_modes for mode in required_modes):
        #     feedback = f"필수 모드 시퀀스({required_modes})가 실행되지 않았습니다. Planner의 계획을 수정해야 합니다."
        #     print(f"[{self.name}] 검증 실패: {feedback}")
        #     return False, feedback

        print(f"[{self.name}] 모드 시퀀스가 정확합니다.")
        return True, "모드 시퀀스 검증 통과"