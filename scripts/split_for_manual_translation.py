#!/usr/bin/env python3
"""
Split PO file into chunks for manual translation by AI.
This script prepares untranslated entries for context-aware manual processing.
"""

import re
import sys
from pathlib import Path

def reset_po_file(po_file_path):
    """Reset all translations in PO file to start fresh."""
    print("Resetting PO file translations to start fresh...")
    
    with open(po_file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Reset all msgstr entries to empty
    reset_content = re.sub(r'(msgstr\s+)"[^"]*"', r'\1""', content)
    reset_content = re.sub(r'(msgstr\s+)""\n(".*?")+', r'\1""', reset_content, flags=re.MULTILINE | re.DOTALL)
    
    with open(po_file_path, 'w', encoding='utf-8') as f:
        f.write(reset_content)
    
    print("PO file reset complete - all translations cleared")

def split_po_for_manual_translation(po_file_path, chunk_size=20):
    """Split PO file into small chunks suitable for Read tool processing."""
    print(f"Splitting PO file into chunks of {chunk_size} entries each...")
    
    with open(po_file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Parse PO file entries
    entries = []
    current_entry = []
    
    for line in content.split('\n'):
        if line.startswith('#:') and current_entry:
            # Start of new entry, save previous
            entries.append('\n'.join(current_entry))
            current_entry = [line]
        else:
            current_entry.append(line)
    
    # Add last entry
    if current_entry:
        entries.append('\n'.join(current_entry))
    
    # Filter out entries that have translations or are empty msgids
    untranslated_entries = []
    for entry in entries:
        if 'msgstr ""' in entry and 'msgid ""' not in entry.split('\n')[-3:]:
            # Check if msgid is not empty
            msgid_match = re.search(r'msgid\s+"([^"]*)"', entry)
            if msgid_match and msgid_match.group(1).strip():
                untranslated_entries.append(entry)
    
    print(f"Found {len(untranslated_entries)} untranslated entries")
    
    # Create chunks directory
    chunks_dir = Path("/tmp/po_chunks")
    chunks_dir.mkdir(exist_ok=True)
    
    # Clear existing chunks
    for chunk_file in chunks_dir.glob("chunk_*.po"):
        chunk_file.unlink()
    
    # Create chunks
    chunk_num = 1
    for i in range(0, len(untranslated_entries), chunk_size):
        chunk_entries = untranslated_entries[i:i + chunk_size]
        chunk_file = chunks_dir / f"chunk_{chunk_num:03d}.po"
        
        with open(chunk_file, 'w', encoding='utf-8') as f:
            f.write("# Chunk for manual translation\n")
            f.write("msgid \"\"\n")
            f.write("msgstr \"\"\n\n")
            
            for entry in chunk_entries:
                f.write(entry + "\n\n")
        
        print(f"Created {chunk_file.name} with {len(chunk_entries)} entries")
        chunk_num += 1
    
    print(f"Created {chunk_num - 1} chunks for manual translation")
    return chunk_num - 1

if __name__ == "__main__":
    po_file_path = "/home/aeon/repos/AutoSDV/book/po/zh-TW.po"
    
    if len(sys.argv) > 1 and sys.argv[1] == "reset":
        reset_po_file(po_file_path)
    
    total_chunks = split_po_for_manual_translation(po_file_path)
    print(f"\nReady for manual translation!")
    print(f"Process chunks in /tmp/po_chunks/ (chunk_001.po to chunk_{total_chunks:03d}.po)")
    print("Use Read tool to examine each chunk and translate manually.")