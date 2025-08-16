# rag_utils/cve_rag.py
from typing import List, Dict, Any
from .cve_database import CVEDatabase
from .build_ragbase import build_faiss_index, save_faiss, embed_texts
from .schema import ChunkRecord, ChunkMetadata, DocumentRecord
import uuid

class CVERAGBuilder:
    def __init__(self, cve_db: CVEDatabase):
        self.cve_db = cve_db
    
    def build_rag_index(self, output_dir: str = "data/rag_sources/cve_index"):
        """CVE 데이터베이스를 RAG 인덱스로 변환"""
        cves = self.cve_db.cves
        
        # CVE를 청크로 변환
        chunks = []
        for cve in cves:
            # CVE 정보를 텍스트로 변환
            cve_text = self._format_cve_for_rag(cve)
            
            # 메타데이터 생성
            metadata = ChunkMetadata(
                section="CVE",
                rule_type="vulnerability",
                keywords=cve['keywords']
            )
            
            # 청크 생성
            chunk = ChunkRecord(
                doc_id=f"cve_{cve['cve_id']}",
                title=cve['cve_id'],
                text=cve_text,
                metadata=metadata
            )
            chunks.append(chunk)
        
        # 임베딩 생성
        texts = [chunk.text for chunk in chunks]
        embeddings = embed_texts(texts)
        
        # FAISS 인덱스 생성
        index = build_faiss_index(chunks, embeddings)
        
        # 저장
        save_faiss(index, output_dir)
        
        print(f"CVE RAG 인덱스 생성 완료: {len(chunks)}개 청크")
        return index
    
    def _format_cve_for_rag(self, cve: Dict[str, Any]) -> str:
        """CVE를 RAG용 텍스트로 변환"""
        text_parts = [
            f"CVE ID: {cve['cve_id']}",
            f"Severity: {cve['severity']}",
            f"Description: {cve['description']}"
        ]
        
        if cve['cvss_score']:
            text_parts.append(f"CVSS Score: {cve['cvss_score']}")
        
        if cve['keywords']:
            text_parts.append(f"Keywords: {', '.join(cve['keywords'])}")
        
        if cve['affected_products']:
            text_parts.append(f"Affected Products: {', '.join(cve['affected_products'])}")
        
        return " | ".join(text_parts)