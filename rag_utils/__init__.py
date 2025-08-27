#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RAG Utilities Module
ROS-LLM 프로젝트를 위한 RAG (Retrieval Augmented Generation) 유틸리티들을 제공합니다.
"""

# Core RAG components
from .build_ragbase import (
    RAGIndex, 
    build_faiss_index, 
    save_faiss, 
    load_faiss, 
    search, 
    embed_texts
)

# Schema definitions
from .schema import (
    ChunkMetadata,
    ChunkRecord, 
    DocumentRecord,
    EmbeddingRecord,
    RetrievalResult,
    generate_id
)

# CWE related modules
from .cwe_database import CWEDatabase, CWEData
from .cwe_rag import CWERAGBuilder, CWERAGSearch
from .cwe_collector import ROSCWECollector as CWECollector

# CVE related modules  
from .cve_rag import CVERAG, CVERAGBuilder
from .cve_validator import CVEValidator

# Security and guidelines
from .security_guidelines import SecurityGuidelineGenerator

# Text processing
from .splitters import (
    split_text_into_paragraphs,
    split_markdown_by_headings, 
    chunk_paragraphs_to_sentences
)

# Configuration
from .config import Config

# External APIs
from .mitre_cwe_api import MITRECWEAPIClient as MitreCWEAPI
from .nvd_api import NVDApiClient as NVDAPI

__all__ = [
    # Core RAG
    'RAGIndex',
    'build_faiss_index',
    'save_faiss', 
    'load_faiss',
    'search',
    'embed_texts',
    
    # Schema
    'ChunkMetadata',
    'ChunkRecord',
    'DocumentRecord', 
    'EmbeddingRecord',
    'RetrievalResult',
    'generate_id',
    
    # CWE
    'CWEDatabase',
    'CWEData',
    'CWERAGBuilder',
    'CWERAGSearch', 
    'CWECollector',
    
    # CVE
    'CVERAG',
    'CVERAGBuilder',
    'CVEValidator',
    
    # Security
    'SecurityGuidelineGenerator',
    
    # Text processing
    'split_text_into_paragraphs',
    'split_markdown_by_headings',
    'chunk_paragraphs_to_sentences',
    
    # Config
    'Config',
    
    # APIs
    'MitreCWEAPI',
    'NVDAPI'
]