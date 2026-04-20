#!/usr/bin/env python3
"""
Extract all images pasted into a Claude Code chat session.

Usage:
    python3 scripts/extract_chat_images.py <session.jsonl> [output_dir]

Defaults:
    output_dir = docs/screenshots/chat_exports

Example:
    python3 scripts/extract_chat_images.py \
        ~/.claude/projects/-home-lars-workspace-apeiron/306a2e58-53ef-489b-b7e2-ae8f1f7cdcf0.jsonl
"""

import base64
import json
import pathlib
import sys


def extract(jsonl_path: pathlib.Path, out_dir: pathlib.Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    n = 0
    for line in jsonl_path.read_text().splitlines():
        if '"type": "image"' not in line and '"type":"image"' not in line:
            continue
        msg = json.loads(line)
        content = msg.get("message", {}).get("content", [])
        if not isinstance(content, list):
            continue
        for block in content:
            if not isinstance(block, dict) or block.get("type") != "image":
                continue
            src = block.get("source", {})
            if src.get("type") != "base64":
                continue
            mt = src.get("media_type", "image/png")
            ext = mt.split("/")[-1]
            fname = out_dir / f"image_{n:02d}.{ext}"
            fname.write_bytes(base64.b64decode(src["data"]))
            print(f"  {fname}")
            n += 1
    return n


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    jsonl_path = pathlib.Path(sys.argv[1])
    out_dir = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else pathlib.Path("docs/screenshots/chat_exports")

    if not jsonl_path.exists():
        print(f"Error: {jsonl_path} not found", file=sys.stderr)
        sys.exit(1)

    print(f"Extracting from {jsonl_path} → {out_dir}")
    n = extract(jsonl_path, out_dir)
    print(f"\nExtracted {n} image(s).")


if __name__ == "__main__":
    main()
