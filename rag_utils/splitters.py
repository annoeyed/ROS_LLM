from __future__ import annotations

from typing import Iterable, List, Tuple


def split_markdown_by_headings(md_text: str) -> List[Tuple[str, str]]:
    """
    Returns list of (section_title, section_text).
    A very simple splitter that treats lines starting with # as section headers.
    The first section may have an empty title.
    """
    lines = md_text.splitlines()
    sections: List[Tuple[str, List[str]]] = []
    current_title = ""
    current_lines: List[str] = []

    def push():
        nonlocal current_title, current_lines
        if current_lines:
            sections.append((current_title.strip(), current_lines))
            current_title = ""
            current_lines = []

    for line in lines:
        if line.lstrip().startswith("#"):
            # new section
            push()
            current_title = line.lstrip("# ")
        else:
            current_lines.append(line)
    push()

    return [(title, "\n".join(body).strip()) for title, body in sections if "".join(body).strip()]


def split_text_into_paragraphs(text: str, min_len: int = 120) -> List[str]:
    paragraphs: List[str] = []
    buf: List[str] = []
    for line in text.splitlines():
        if not line.strip():
            paragraph = " ".join(buf).strip()
            if paragraph:
                paragraphs.append(paragraph)
            buf = []
        else:
            buf.append(line.strip())
    if buf:
        paragraph = " ".join(buf).strip()
        if paragraph:
            paragraphs.append(paragraph)

    # merge short paragraphs
    merged: List[str] = []
    for p in paragraphs:
        if merged and (len(merged[-1]) < min_len or len(p) < min_len):
            merged[-1] = (merged[-1] + " " + p).strip()
        else:
            merged.append(p)
    return merged


def sentence_split(text: str) -> List[str]:
    # naive sentence splitter that handles periods/question/exclamation
    import re

    parts = re.split(r"(?<=[.!?])\s+", text.strip())
    return [p.strip() for p in parts if p.strip()]


def chunk_paragraphs_to_sentences(paragraphs: Iterable[str], max_tokens: int = 256) -> List[str]:
    """
    Very naive token estimation: 1 token ~= 4 chars (English). For KR/EN mixed text, we keep it simple.
    """
    chunks: List[str] = []
    buf: List[str] = []
    buf_len = 0
    for p in paragraphs:
        for sent in sentence_split(p):
            sent_len = len(sent)
            if buf_len + sent_len > max_tokens * 4 and buf:
                chunks.append(" ".join(buf))
                buf = []
                buf_len = 0
            buf.append(sent)
            buf_len += sent_len + 1
        if buf:
            chunks.append(" ".join(buf))
            buf = []
            buf_len = 0
    return chunks


