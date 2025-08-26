# oracles/param_oracle.py
from .base_oracle import BaseOracle
import re

class ParamOracle(BaseOracle):
    """파라미터의 안전 범위를 검증하는 Oracle입니다."""
    def __init__(self):
        super().__init__("ParamOracle")

    def verify(self, code: str, simulation_result: dict) -> (bool, str):
        print(f"[{self.name}] 파라미터 검증 시작...")
        # 예시: 코드 내에서 속도 관련 파라미터가 10.0을 초과하는지 검사
        # 실제로는 simulation_result를 분석해야 합니다.
        speed_match = re.search(r"speed\s*=\s*(\d+\.?\d*)", code)
        if speed_match:
            speed = float(speed_match.group(1))
            if speed > 10.0:
                feedback = f"속도 파라미터({speed})가 안전 범위(10.0)를 초과했습니다. 코드를 수정하여 속도를 낮춰주세요."
                print(f"[{self.name}] 검증 실패: {feedback}")
                return False, feedback
        
        print(f"[{self.name}] 모든 파라미터가 안전 범위 내에 있습니다.")
        return True, "파라미터 검증 통과"