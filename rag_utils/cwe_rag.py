#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CWE RAG 시스템
CWE 데이터베이스를 기반으로 보안 취약점 정보를 검색할 수 있는 RAG 시스템
"""

import json
import os
from typing import List, Dict, Any, Optional
from .cwe_database import CWEDatabase
from .build_ragbase import build_faiss_index, save_faiss, embed_texts, search, load_faiss
from .schema import ChunkRecord, ChunkMetadata, DocumentRecord, RetrievalResult
import uuid

class CWERAGBuilder:
    def __init__(self, cwe_db: CWEDatabase):
        self.cwe_db = cwe_db
    
    def build_rag_index(self, output_dir: str = "data/rag_sources/cwe_index"):
        """Convert CWE database to RAG index"""
        cwes = self.cwe_db.cwes
        
        print(f"Starting CWE RAG index construction: {len(cwes)} CWEs")
        
        # Convert CWEs to chunks
        chunks = []
        for cwe in cwes:
            # Convert CWE information to text
            cwe_text = self._format_cwe_for_rag(cwe)
            
            # Create metadata
            metadata = ChunkMetadata(
                section="CWE",
                rule_type="vulnerability",
                keywords=self._extract_keywords(cwe)
            )
            
            # Create chunk
            chunk = ChunkRecord(
                doc_id=f"cwe_{cwe['cwe_id']}",
                title=f"CWE-{cwe['cwe_id']}: {cwe['name']}",
                text=cwe_text,
                metadata=metadata
            )
            chunks.append(chunk)
        
        print(f"Chunk creation completed: {len(chunks)} chunks")
        
        # Generate embeddings
        texts = [chunk.text for chunk in chunks]
        print("Generating embeddings...")
        embeddings = embed_texts(texts)
        
        # Create FAISS index
        print("Creating FAISS index...")
        index = build_faiss_index(chunks, embeddings)
        
        # Save
        print(f"Saving index to: {output_dir}")
        save_faiss(index, output_dir)
        
        print(f"CWE RAG index creation completed: {len(chunks)} chunks")
        return index
    
    def _format_cwe_for_rag(self, cwe: Dict[str, Any]) -> str:
        """Convert CWE to RAG text format"""
        text_parts = [
            f"CWE ID: CWE-{cwe['cwe_id']}",
            f"Name: {cwe['name']}",
            f"Description: {cwe['description']}"
        ]
        
        # Extract additional information from raw_data
        raw_data = cwe.get('raw_data', {})
        
        if raw_data.get('abstraction'):
            text_parts.append(f"Abstraction: {raw_data['abstraction']}")
        
        if raw_data.get('structure'):
            text_parts.append(f"Structure: {raw_data['structure']}")
        
        if raw_data.get('status'):
            text_parts.append(f"Status: {raw_data['status']}")
        
        # Add mitigations
        if raw_data.get('mitigations'):
            mitigations_text = " | ".join([m.get('description', '') for m in raw_data['mitigations']])
            if mitigations_text:
                text_parts.append(f"Mitigations: {mitigations_text}")
        
        # Add examples
        if raw_data.get('examples'):
            examples_text = " | ".join([ex.get('description', '') for ex in raw_data['examples']])
            if examples_text:
                text_parts.append(f"Examples: {examples_text}")
        
        return " | ".join(text_parts)
    
    def _extract_keywords(self, cwe: Dict[str, Any]) -> List[str]:
        """Extract keywords from CWE"""
        keywords = []
        
        # 기본 키워드
        text = f"{cwe['name']} {cwe['description']}".lower()
        
        # ROS 관련 키워드
        ros_keywords = ['ros', 'dds', 'topic', 'service', 'parameter', 'node', 'publisher', 'subscriber']
        for kw in ros_keywords:
            if kw in text:
                keywords.append(kw.upper())
        
        # 보안 관련 키워드
        security_keywords = ['authentication', 'authorization', 'encryption', 'tls', 'ssl', 'permission', 'access control']
        for kw in security_keywords:
            if kw in text:
                keywords.append(kw)
        
        # CWE 카테고리
        if cwe.get('category'):
            keywords.append(cwe['category'])
        
        return keywords

class CWERAGSearch:
    def __init__(self, index_dir: str = "data/rag_sources/cwe_index"):
        """CWE RAG 검색 시스템 초기화"""
        self.index_dir = index_dir
        self.index = None
        self._load_index()
    
    def _load_index(self):
        """RAG 인덱스 로드"""
        try:
            self.index = load_faiss(self.index_dir)
            print(f"CWE RAG 인덱스 로드 완료: {len(self.index.chunk_id_to_chunk)}개 청크")
        except Exception as e:
            print(f"인덱스 로드 실패: {e}")
            self.index = None
    
    def search(self, query: str, top_k: int = 5) -> List[RetrievalResult]:
        """CWE 검색"""
        if not self.index:
            print("인덱스가 로드되지 않았습니다.")
            return []
        
        print(f"검색 쿼리: '{query}'")
        results = search(self.index, query, top_k)
        
        print(f"검색 결과: {len(results)}개")
        for i, result in enumerate(results):
            print(f"{i+1}. CWE-{result.chunk.title.split(':')[0].split('CWE-')[1]} (점수: {result.score:.3f})")
            print(f"   {result.chunk.title.split(':')[1].strip()}")
            print(f"   {result.chunk.text[:200]}...")
            print()
        
        return results
    
    def search_by_cwe_id(self, cwe_id: str) -> Optional[RetrievalResult]:
        """특정 CWE ID로 검색"""
        if not self.index:
            return None
        
        # CWE ID 정규화
        if not cwe_id.startswith('CWE-'):
            cwe_id = f"CWE-{cwe_id}"
        
        # 정확한 매칭 검색
        for chunk in self.index.chunk_id_to_chunk.values():
            if chunk.title.startswith(cwe_id):
                return RetrievalResult(chunk=chunk, score=1.0)
        
        return None
    
    def search_by_category(self, category: str, top_k: int = 10) -> List[RetrievalResult]:
        """카테고리별 검색"""
        if not self.index:
            return []
        
        # 카테고리 키워드로 검색
        query = f"category: {category}"
        return self.search(query, top_k)
    
    def search_by_component(self, component: str, top_k: int = 10) -> List[RetrievalResult]:
        """ROS 컴포넌트별 검색"""
        if not self.index:
            return []
        
        # 컴포넌트 키워드로 검색
        query = f"ROS component: {component}"
        return self.search(query, top_k)

def test_cwe_rag():
    """CWE RAG 시스템 테스트"""
    print("=== CWE RAG 시스템 테스트 시작 ===")
    
    # 1. CWE 데이터베이스 로드
    try:
        from .cwe_database import CWEDatabase
        cwe_db = CWEDatabase()
        cwe_db.load_database()
        print(f"CWE 데이터베이스 로드 완료: {len(cwe_db.cwes)}개 CWE")
    except Exception as e:
        print(f"CWE 데이터베이스 로드 실패: {e}")
        return
    
    # 2. RAG 인덱스 구축
    try:
        builder = CWERAGBuilder(cwe_db)
        index = builder.build_rag_index()
        print("RAG 인덱스 구축 완료")
    except Exception as e:
        print(f"RAG 인덱스 구축 실패: {e}")
        return
    
    # 3. 검색 테스트
    try:
        searcher = CWERAGSearch()
        
        # 기본 검색 테스트
        print("\n--- 기본 검색 테스트 ---")
        searcher.search("authentication", 3)
        
        # CWE ID 검색 테스트
        print("\n--- CWE ID 검색 테스트 ---")
        result = searcher.search_by_cwe_id("287")
        if result:
            print(f"찾은 CWE: {result.chunk.title}")
        
        # 카테고리 검색 테스트
        print("\n--- 카테고리 검색 테스트 ---")
        searcher.search_by_category("input validation", 3)
        
        print("\n=== CWE RAG 시스템 테스트 완료 ===")
        
    except Exception as e:
        print(f"검색 테스트 실패: {e}")

if __name__ == "__main__":
    test_cwe_rag()
