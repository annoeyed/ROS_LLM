#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Base Agent Class
Defines the basic structure and common functionality that all Agents inherit
"""

import json
import logging
from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, asdict

@dataclass
class AgentMessage:
    """Message structure for inter-agent communication"""
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
    """Definition of tasks that an Agent performs"""
    task_id: str
    task_type: str
    description: str
    parameters: Dict[str, Any]
    priority: int = 1  # 1: Low, 2: Medium, 3: High, 4: Critical
    status: str = 'pending'  # pending, running, completed, failed
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

class BaseAgent(ABC):
    """Base class for all Agents"""
    
    def __init__(self, agent_id: str, agent_name: str):
        self.agent_id = agent_id
        self.agent_name = agent_name
        self.status = 'idle'  # idle, busy, error, offline
        self.message_queue: List[AgentMessage] = []
        self.task_history: List[AgentTask] = []
        self.current_task: Optional[AgentTask] = None
        
        # Logging configuration
        self.logger = logging.getLogger(f"Agent.{agent_name}")
        self.logger.setLevel(logging.INFO)
        
        # Agent initialization
        self._initialize()
    
    def _initialize(self):
        """Agent initialization - implemented in subclasses"""
        self.logger.info(f"Agent {self.agent_name} initialization complete")
    
    @abstractmethod
    def process_message(self, message: AgentMessage) -> AgentMessage:
        """Message processing - implemented in subclasses"""
        pass
    
    @abstractmethod
    def execute_task(self, task: AgentTask) -> Dict[str, Any]:
        """Task execution - implemented in subclasses"""
        pass
    
    def send_message(self, receiver: str, message_type: str, content: Dict[str, Any]) -> AgentMessage:
        """Send message"""
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
        
        self.logger.info(f"Message sent: {message_type} type message to {receiver}")
        return message
    
    def receive_message(self, message: AgentMessage):
        """Receive message"""
        self.message_queue.append(message)
        self.logger.info(f"Message received: {message.message_type} type message from {message.sender}")
        
        # Process message
        try:
            response = self.process_message(message)
            return response
        except Exception as e:
            self.logger.error(f"Message processing failed: {e}")
            return self.send_message(
                message.sender,
                'error',
                {'error': str(e), 'original_message': message.content}
            )
    
    def add_task(self, task: AgentTask):
        """Add task"""
        self.task_history.append(task)
        self.logger.info(f"Task added: {task.task_type} - {task.description}")
    
    def get_status(self) -> Dict[str, Any]:
        """Return Agent status"""
        return {
            'agent_id': self.agent_id,
            'agent_name': self.agent_name,
            'status': self.status,
            'message_queue_length': len(self.message_queue),
            'task_history_length': len(self.task_history),
            'current_task': self.current_task.task_id if self.current_task else None
        }
    
    def get_task_status(self, task_id: str) -> Optional[AgentTask]:
        """Query specific task status"""
        for task in self.task_history:
            if task.task_id == task_id:
                return task
        return None
    
    def update_task_status(self, task_id: str, status: str, result: Optional[Dict[str, Any]] = None, error: Optional[str] = None):
        """Update task status"""
        task = self.get_task_status(task_id)
        if task:
            task.status = status
            if result:
                task.result = result
            if error:
                task.error = error
            self.logger.info(f"Task status updated: {task_id} -> {status}")
    
    def clear_message_queue(self):
        """Clear message queue"""
        self.message_queue.clear()
        self.logger.info("Message queue cleared")
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Return performance metrics"""
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
