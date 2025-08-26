from .base_oracle import BaseOracle

class SafetyOracle(BaseOracle):
    """비상 절차, 금지 API 사용 등을 검증하는 Oracle입니다."""
    def __init__(self):
        super().__init__("SafetyOracle")

    def verify(self, code: str, simulation_result: dict) -> (bool, str):
        print(f"[{self.name}] 안전 정책 검증 시작...")
        # 예시: 금지된 'os.system' 함수 사용 여부 검사
        if "os.system" in code:
            feedback = "'os.system'과 같이 잠재적으로 위험한 API가 사용되었습니다. 더 안전한 subprocess 모듈을 사용하세요."
            print(f"[{self.name}] 검증 실패: {feedback}")
            return False, feedback
            
        print(f"[{self.name}] 안전 정책을 준수합니다.")
        return True, "안전 정책 검증 통과"