# oracles/base_oracle.py
from abc import ABC, abstractmethod

class BaseOracle(ABC):
    """Oracle의 기본 인터페이스 역할을 하는 추상 기본 클래스입니다."""
    def __init__(self, name):
        self.name = name

    @abstractmethod
    def verify(self, code: str, simulation_result: dict) -> (bool, str):
        """
        주어진 코드를 검증합니다.
        :param code: 검증할 소스 코드
        :param simulation_result: 시뮬레이션 결과 데이터
        :return: (검증 통과 여부, 피드백 메시지) 튜플
        """
        pass