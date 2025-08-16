from __future__ import annotations

from typing import Any, Dict, List, Optional
from pydantic import BaseModel, Field
import uuid


def generate_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


class DocumentRecord(BaseModel):
    doc_id: str = Field(default_factory=lambda: generate_id("doc"))
    title: str
    source_path: str
    source_url: Optional[str] = None


class ChunkMetadata(BaseModel):
    section: Optional[str] = None
    rule_type: Optional[str] = None
    keywords: List[str] = Field(default_factory=list)


class ChunkRecord(BaseModel):
    chunk_id: str = Field(default_factory=lambda: generate_id("chunk"))
    doc_id: str
    title: str
    text: str
    metadata: ChunkMetadata


class EmbeddingRecord(BaseModel):
    chunk_id: str
    vector: List[float]


class RetrievalResult(BaseModel):
    chunk: ChunkRecord
    score: float


class TaskStep(BaseModel):
    type: str
    params: Dict[str, Any] = Field(default_factory=dict)


class TaskSpec(BaseModel):
    steps: List[TaskStep]
    tolerances: Dict[str, Any] = Field(default_factory=dict)
    safety_limits: Dict[str, Any] = Field(default_factory=dict)


class GuardFinding(BaseModel):
    severity: str  # info|low|medium|high|critical
    title: str
    rationale: str
    evidence_snippets: List[str] = Field(default_factory=list)
    references: List[str] = Field(default_factory=list)


class GuardReport(BaseModel):
    passed: bool
    findings: List[GuardFinding] = Field(default_factory=list)
    summary: str = ""


