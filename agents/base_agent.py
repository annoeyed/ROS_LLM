#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Agent 기본 클래스
모든 Agent가 상속받는 기본 구조와 공통 기능을 정의
"""

import json
import logging
from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, asdict

@dataclass
class AgentMessage:
    """Agent 간 통신을 위한 메시지 구조"""
    sender: str
    receiver: str
    message_type: str  # 'request', 'response', 'notification', 'error'
    content: Dict[str, Any]
    timestamp: float
    message_id: str
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
    
    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2)

@dataclass
class AgentTask:
    """Agent가 수행할 작업 정의"""
    task_id: str
    task_type: str
    description: str
    parameters: Dict[str, Any]
    priority: int = 1  # 1: Low, 2: Medium, 3: High, 4: Critical
    status: str = 'pending'  # pending, running, completed, failed
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

class BaseAgent(ABC):
    """모든 Agent의 기본 클래스"""
    
    def __init__(self, agent_id: str, agent_name: str):
        self.agent_id = agent_id
        self.agent_name = agent_name
        self.status = 'idle'  # idle, busy, error, offline
        self.message_queue: List[AgentMessage] = []
        self.task_history: List[AgentTask] = []
        self.current_task: Optional[AgentTask] = None
        
        # 로깅 설정
        self.logger = logging.getLogger(f"Agent.{agent_name}")
        self.logger.setLevel(logging.INFO)
        
        # Agent 초기화
        self._initialize()
    
    def _initialize(self):
        """Agent 초기화 - 하위 클래스에서 구현"""
        self.logger.info(f"Agent {self.agent_name} 초기화 완료")
    
    @abstractmethod
    def process_message(self, message: AgentMessage) -> AgentMessage:
        """메시지 처리 - 하위 클래스에서 구현"""
        pass
    
    @abstractmethod
    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """작업 실행 - 하위 클래스에서 구현"""
        pass
    
    def send_message(self, receiver: str, message_type: str, content: Dict[str, Any]) -> AgentMessage:
        """메시지 전송"""
        import time
        import uuid
        
        message = AgentMessage(
            sender=self.agent_id,
            receiver=receiver,
            message_type=message_type,
            content=content,
            timestamp=time.time(),
            message_id=str(uuid.uuid4())
        )
        
        self.logger.info(f"메시지 전송: {receiver}에게 {message_type} 타입 메시지")
        return message
    
    def receive_message(self, message: AgentMessage):
        """메시지 수신"""
        self.message_queue.append(message)
        self.logger.info(f"메시지 수신: {message.sender}로부터 {message.message_type} 타입 메시지")
        
        # 메시지 처리
        try:
            response = self.process_message(message)
            return response
        except Exception as e:
            self.logger.error(f"메시지 처리 실패: {e}")
            return self.send_message(
                message.sender,
                'error',
                {'error': str(e), 'original_message': message.content}
            )
    
    def add_task(self, task: AgentTask):
        """작업 추가"""
        self.task_history.append(task)
        self.logger.info(f"작업 추가: {task.task_type} - {task.description}")
    
    def get_status(self) -> Dict[str, Any]:
        """Agent 상태 반환"""
        return {
            'agent_id': self.agent_id,
            'agent_name': self.agent_name,
            'status': self.status,
            'message_queue_length': len(self.message_queue),
            'task_history_length': len(self.task_history),
            'current_task': self.current_task.task_id if self.current_task else None
        }
    
    def get_task_status(self, task_id: str) -> Optional[AgentTask]:
        """특정 작업 상태 조회"""
        for task in self.task_history:
            if task.task_id == task_id:
                return task
        return None
    
    def update_task_status(self, task_id: str, status: str, result: Optional[Dict[str, Any]] = None, error: Optional[str] = None):
        """작업 상태 업데이트"""
        task = self.get_task_status(task_id)
        if task:
            task.status = status
            if result:
                task.result = result
            if error:
                task.error = error
            self.logger.info(f"작업 상태 업데이트: {task_id} -> {status}")
    
    def clear_message_queue(self):
        """메시지 큐 정리"""
        self.message_queue.clear()
        self.logger.info("메시지 큐 정리 완료")
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """성능 메트릭 반환"""
        completed_tasks = [t for t in self.task_history if t.status == 'completed']
        failed_tasks = [t for t in self.task_history if t.status == 'failed']
        
        total_tasks = len(self.task_history)
        success_rate = len(completed_tasks) / total_tasks if total_tasks > 0 else 0
        
        return {
            'total_tasks': total_tasks,
            'completed_tasks': len(completed_tasks),
            'failed_tasks': len(failed_tasks),
            'success_rate': success_rate,
            'pending_messages': len(self.message_queue)
        }
    
    def __str__(self) -> str:
        return f"{self.agent_name}({self.agent_id}) - {self.status}"
    
    def __repr__(self) -> str:
        return self.__str__()
