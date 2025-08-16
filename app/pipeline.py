# app/pipeline.py
from __future__ import annotations

import json
import os
from typing import List

from rag_utils.build_ragbase import build_faiss_index, build_from_markdown, save_faiss, load_faiss, search
from rag_utils.schema import GuardFinding, GuardReport, TaskSpec, TaskStep
from rag_utils.cve_database import CVEDatabase
from rag_utils.cve_rag import CVERAGBuilder

def load_google_doc_export(md_path: str) -> str:
    with open(md_path, "r", encoding="utf-8") as f:
        return f.read()

def ingest_markdown(md_text: str, title: str, source_path: str, source_url: str | None, out_dir: str) -> None:
    chunks, embeddings = build_from_markdown(md_text, title=title, source_path=source_path, source_url=source_url)
    index = build_faiss_index(chunks, embeddings)
    save_faiss(index, out_dir)

def run_guard(index_dir: str, task: TaskSpec) -> GuardReport:
    index = load_faiss(index_dir)
    # build a few queries from the task
    queries: List[str] = []
    step_types = ", ".join(sorted(set(s.type for s in task.steps)))
    queries.append(f"ROS security guidelines for steps: {step_types}")
    if "takeoff" in step_types:
        queries.append("drone takeoff safety limits and failsafe")
    if task.safety_limits:
        queries.append("ROS safety limits configuration best practices")

    results = []
    for q in queries:
        results.extend(search(index, q, top_k=5))

    findings: List[GuardFinding] = []
    risky_keywords = [
        "disable security", "anonymous", "no auth", "unencrypted", "허가 없이", "비암호화", "open access", "wide permissions",
    ]
    evidence = []
    for r in results:
        text_lower = r.chunk.text.lower()
        if any(k in text_lower for k in [k.lower() for k in risky_keywords]):
            evidence.append(r.chunk.text)

    if evidence:
        findings.append(GuardFinding(
            severity="high",
            title="잠재적 보안 리스크 감지",
            rationale="RAG 결과에서 위험 키워드가 포함된 스니펫이 발견됨",
            evidence_snippets=evidence[:5],
            references=[
                "https://docs.google.com/document/d/12JN2UJ5Mu4cXhWvYeoRdewMYnDK7DtaD/edit",
            ],
        ))

    passed = len(findings) == 0
    summary = "통과" if passed else "보안 리스크 발견"
    return GuardReport(passed=passed, findings=findings, summary=summary)

def run_guard_with_cve(cve_index_dir: str, task: TaskSpec) -> GuardReport:
    """CVE RAG 인덱스를 사용한 가드 실행"""
    from rag_utils.build_ragbase import load_faiss, search
    
    index = load_faiss(cve_index_dir)
    
    # 태스크에서 위험 키워드 추출
    risky_patterns = []
    for step in task.steps:
        step_type = step.type.lower()
        if 'anonymous' in step_type:
            risky_patterns.append('anonymous')
        if 'unencrypted' in step_type:
            risky_patterns.append('unencrypted')
    
    # CVE RAG에서 관련 보안 정보 검색
    findings = []
    for pattern in risky_patterns:
        results = search(index, pattern, top_k=3)
        for result in results:
            if result.score > 0.5:  # 임계값
                findings.append(GuardFinding(
                    severity="high",
                    title=f"CVE 관련 보안 위험: {pattern}",
                    rationale=f"CVE 데이터베이스에서 {pattern} 관련 취약점 발견",
                    evidence_snippets=[result.chunk.text],
                    references=[f"CVE: {result.chunk.title}"]
                ))
    
    passed = len(findings) == 0
    summary = "통과" if passed else f"{len(findings)}개 보안 위험 발견"
    
    return GuardReport(passed=passed, findings=findings, summary=summary)

def generate_code_from_task(task: TaskSpec) -> str:
    # 간단한 MVP 코드 생성기 (의사코드 수준)
    lines: List[str] = []
    lines.append("# Auto-generated mission script (MVP)\n")
    lines.append("def run_mission():")
    lines.append("    # Safety limits")
    for k, v in task.safety_limits.items():
        lines.append(f"    safety_{k} = {repr(v)}")
    lines.append("    # Steps")
    for step in task.steps:
        st = step.type
        if st == "takeoff":
            lines.append("    print('Takeoff')")
        elif st == "waypoint":
            lat = step.params.get("lat", 0)
            lon = step.params.get("lon", 0)
            lines.append(f"    print('Goto waypoint', {lat}, {lon})")
        elif st == "loiter":
            sec = step.params.get("seconds", 10)
            lines.append(f"    print('Loiter for {sec}s')")
        elif st == "land":
            lines.append("    print('Land')")
        else:
            lines.append(f"    print('Unknown step: {st}')")
    lines.append("\nif __name__ == '__main__':\n    run_mission()\n")
    return "\n".join(lines)

def revise_code_based_on_guard(code_text: str, report: GuardReport) -> str:
    if report.passed:
        return code_text
    # 간단한 수정: 위험 키워드가 있으면 로그로 경고 출력 추가
    warning = "# WARNING: Guard reported security risks. Review required.\n"
    return warning + code_text

def simulate_oracle_unittests(code_text: str, task: TaskSpec) -> dict:
    """아주 단순한 규칙 기반 시뮬 오라클.
    - takeoff가 있으면 반드시 land가 있어야 함
    - safety_limits에 geofence가 양수여야 함
    - steps는 최소 2개 이상
    """
    messages = []
    step_types = [s.type for s in task.steps]
    passed = True

    if "takeoff" in step_types and "land" not in step_types:
        passed = False
        messages.append("takeoff가 있으나 land가 없습니다")
    geo = task.safety_limits.get("geofence", None)
    if geo is None or not isinstance(geo, (int, float)) or geo <= 0:
        passed = False
        messages.append("geofence 설정이 없거나 0 이하입니다")
    if len(step_types) < 2:
        passed = False
        messages.append("step이 2개 미만입니다")

    return {"passed": passed, "messages": messages}

def main() -> None:
    # 1) CVE 수집 및 RAG 인덱스 구축
    print("=== CVE 수집 및 RAG 인덱스 구축 ===")
    from rag_utils.nvd_api import NVDApiClient
    
    client = NVDApiClient()
    cves = client.search_ros_cves()
    
    db = CVEDatabase()
    result = db.add_cves(cves, 'NVD API')
    print(f"CVE 데이터베이스 업데이트: {result}")
    
    # CVE RAG 인덱스 구축
    cve_rag = CVERAGBuilder(db)
    cve_rag.build_rag_index()
    
    # 2) 태스크 스펙 예시 (위험한 태스크로 테스트)
    task = TaskSpec(
        steps=[
            TaskStep(type="anonymous_node"),
            TaskStep(type="unencrypted_topic"),
        ],
        tolerances={"pos": 1.0},
        safety_limits={"geofence": 1000},
    )

    # 3) CVE RAG로 가드 실행
    print("\n=== CVE RAG 가드 테스트 ===")
    report = run_guard_with_cve('data/rag_sources/cve_index', task)
    print("=== Guard Report ===")
    print(json.dumps(report.model_dump(), ensure_ascii=False, indent=2))

    # 4) 코드 생성 → 가드 → (필요 시) 수정 → 재가드 → 시뮬 오라클 유닛테스트
    code_v1 = generate_code_from_task(task)
    if not report.passed:
        code_v2 = revise_code_based_on_guard(code_v1, report)
        final_code = code_v2
    else:
        final_code = code_v1

    print("\n=== Generated Code (MVP) ===")
    print(final_code)

    sim = simulate_oracle_unittests(final_code, task)
    print("\n=== Sim Oracle UnitTests ===")
    print(json.dumps(sim, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()