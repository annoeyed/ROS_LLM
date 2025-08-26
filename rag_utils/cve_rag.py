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
        """Convert CVE database to RAG index"""
        cves = self.cve_db.cves
        
        # Convert CVEs to chunks
        chunks = []
        for cve in cves:
            # Convert CVE information to text
            cve_text = self._format_cve_for_rag(cve)
            
            # Create metadata
            metadata = ChunkMetadata(
                section="CVE",
                rule_type="vulnerability",
                keywords=cve['keywords']
            )
            
            # Create chunk
            chunk = ChunkRecord(
                doc_id=f"cve_{cve['cve_id']}",
                title=cve['cve_id'],
                text=cve_text,
                metadata=metadata
            )
            chunks.append(chunk)
        
        # Generate embeddings
        texts = [chunk.text for chunk in chunks]
        embeddings = embed_texts(texts)
        
        # Create FAISS index
        index = build_faiss_index(chunks, embeddings)
        
        # Save
        save_faiss(index, output_dir)
        
        print(f"CVE RAG index creation completed: {len(chunks)} chunks")
        return index
    
    def _format_cve_for_rag(self, cve: Dict[str, Any]) -> str:
        """Convert CVE to RAG text format"""
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