from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .schema import ChunkMetadata, ChunkRecord, DocumentRecord, EmbeddingRecord, RetrievalResult
from .splitters import chunk_paragraphs_to_sentences, split_markdown_by_headings, split_text_into_paragraphs


try:
    import faiss  # type: ignore
    FAISS_AVAILABLE = True
except Exception:
    FAISS_AVAILABLE = False


@dataclass
class RAGIndex:
    vectors: Optional["faiss.IndexFlatIP"]
    chunk_id_to_chunk: Dict[str, ChunkRecord]
    dim: int


def l2_normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True) + 1e-9
    return v / n


def embed_texts(texts: List[str]) -> np.ndarray:
    """
    Placeholder embedding using simple bag-of-chars hashing for MVP.
    Replace with sentence-transformers or OpenAI embeddings later.
    """
    dim = 384
    rng = np.random.RandomState(42)
    char_map = {c: rng.randn(dim) for c in (list("abcdefghijklmnopqrstuvwxyz0123456789-_.:# ")+['한','글','테','스','트'])}
    vectors = []
    for t in texts:
        v = np.zeros(dim)
        for ch in t.lower():
            if ch in char_map:
                v += char_map[ch]
        vectors.append(v)
    arr = np.vstack(vectors).astype("float32")
    arr = l2_normalize(arr)
    return arr


def infer_metadata(section_title: str, text: str) -> ChunkMetadata:
    title_lower = section_title.lower()
    rule_type = None
    if any(k in title_lower for k in ["mitigation", "대응", "방지", "완화", "guide", "가이드"]):
        rule_type = "guideline"
    elif any(k in title_lower for k in ["cve", "취약점", "vulnerability"]):
        rule_type = "cve"
    keywords = []
    for kw in ["ROS", "DDS", "auth", "encryption", "TLS", "topic", "service", "parameter", "permissions", "시뮬", "안전"]:
        if kw.lower() in text.lower() or kw.lower() in title_lower:
            keywords.append(kw)
    return ChunkMetadata(section=section_title or None, rule_type=rule_type, keywords=keywords)


def build_from_markdown(md_text: str, title: str, source_path: str, source_url: Optional[str] = None) -> Tuple[List[ChunkRecord], np.ndarray]:
    doc = DocumentRecord(title=title, source_path=source_path, source_url=source_url)
    sections = split_markdown_by_headings(md_text)
    if not sections:
        sections = [(title, md_text)]

    chunks: List[ChunkRecord] = []
    for section_title, section_text in sections:
        paragraphs = split_text_into_paragraphs(section_text)
        sentence_chunks = chunk_paragraphs_to_sentences(paragraphs)
        for ch_text in sentence_chunks:
            meta = infer_metadata(section_title, ch_text)
            chunks.append(ChunkRecord(doc_id=doc.doc_id, title=title, text=ch_text, metadata=meta))

    embeddings = embed_texts([c.text for c in chunks])
    return chunks, embeddings


def save_faiss(index: RAGIndex, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    mapping_path = os.path.join(out_dir, "chunks.json")
    vectors_path = os.path.join(out_dir, "vectors.faiss")
    order_path = os.path.join(out_dir, "order.json")

    with open(mapping_path, "w", encoding="utf-8") as f:
        json.dump([c.model_dump() for c in index.chunk_id_to_chunk.values()], f, ensure_ascii=False, indent=2)

    # save order of vectors → chunk_ids
    order = getattr(index, "_order", [])
    with open(order_path, "w", encoding="utf-8") as f:
        json.dump(order, f, ensure_ascii=False)

    if FAISS_AVAILABLE and index.vectors is not None:
        faiss.write_index(index.vectors, vectors_path)  # type: ignore
    else:
        # Save as raw numpy fallback
        np.save(os.path.join(out_dir, "vectors.npy"), getattr(index, "_vectors", None))


def load_faiss(in_dir: str) -> RAGIndex:
    mapping_path = os.path.join(in_dir, "chunks.json")
    with open(mapping_path, "r", encoding="utf-8") as f:
        chunk_list = json.load(f)
    chunks = [ChunkRecord(**c) for c in chunk_list]
    chunk_id_to_chunk = {c.chunk_id: c for c in chunks}
    dim = 384
    index = None
    # load order if present, else fall back to list order
    order_path = os.path.join(in_dir, "order.json")
    if os.path.exists(order_path):
        with open(order_path, "r", encoding="utf-8") as f:
            order = json.load(f)
    else:
        order = [c.chunk_id for c in chunks]
    if FAISS_AVAILABLE and os.path.exists(os.path.join(in_dir, "vectors.faiss")):
        index = faiss.read_index(os.path.join(in_dir, "vectors.faiss"))  # type: ignore
        rag_index = RAGIndex(vectors=index, chunk_id_to_chunk=chunk_id_to_chunk, dim=dim)
    else:
        vec_path = os.path.join(in_dir, "vectors.npy")
        vectors = np.load(vec_path) if os.path.exists(vec_path) else np.zeros((0, dim), dtype=np.float32)
        rag_index = RAGIndex(vectors=None, chunk_id_to_chunk=chunk_id_to_chunk, dim=dim)
        setattr(rag_index, "_vectors", vectors)
    setattr(rag_index, "_order", order)
    return rag_index


def build_faiss_index(chunks: List[ChunkRecord], embeddings: np.ndarray) -> RAGIndex:
    dim = embeddings.shape[1]
    if FAISS_AVAILABLE:
        index = faiss.IndexFlatIP(dim)  # type: ignore
        index.add(embeddings)
        # store ids implicitly by ordering
        rag_index = RAGIndex(vectors=index, chunk_id_to_chunk={c.chunk_id: c for c in chunks}, dim=dim)
    else:
        rag_index = RAGIndex(vectors=None, chunk_id_to_chunk={c.chunk_id: c for c in chunks}, dim=dim)
        setattr(rag_index, "_vectors", embeddings)
    setattr(rag_index, "_order", [c.chunk_id for c in chunks])
    return rag_index


def search(index: RAGIndex, query: str, top_k: int = 5) -> List[RetrievalResult]:
    q = embed_texts([query]).astype("float32")
    if FAISS_AVAILABLE and index.vectors is not None:
        D, I = index.vectors.search(q, top_k)  # type: ignore
        scores = D[0].tolist()
        idxs = I[0].tolist()
    else:
        vectors = getattr(index, "_vectors")
        # cosine similarity over normalized vectors
        sims = (q @ vectors.T)[0]
        idxs = np.argsort(-sims)[:top_k]
        scores = sims[idxs].tolist()

    order: List[str] = getattr(index, "_order")
    results: List[RetrievalResult] = []
    for i, score in zip(idxs, scores):
        if i < 0 or i >= len(order):
            continue
        cid = order[i]
        chunk = index.chunk_id_to_chunk[cid]
        results.append(RetrievalResult(chunk=chunk, score=float(score)))
    return results


