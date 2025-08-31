#!/usr/bin/env python3
"""
Merge manually translated chunks back into the main PO file.
This script collects AI-translated chunks and updates the main PO file.
Part of the manual translation workflow where AI processes chunks one-by-one.
"""

import re
import sys
from pathlib import Path

def extract_translations_from_chunk(chunk_file):
    """Extract msgid/msgstr pairs from a translated chunk."""
    translations = {}
    
    with open(chunk_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Split into entries by #: markers
    entries = re.split(r'\n(?=#:)', content)
    
    for entry in entries:
        if not entry.strip():
            continue
            
        # Extract msgid and msgstr from each entry
        msgid_match = re.search(r'msgid\s+"([^"]*)"', entry)
        msgstr_match = re.search(r'msgstr\s+"([^"]*)"', entry)
        
        if msgid_match and msgstr_match:
            msgid = msgid_match.group(1)
            msgstr = msgstr_match.group(1)
            
            if msgstr.strip():  # Only include non-empty translations
                translations[msgid] = msgstr
    
    return translations

def merge_translations_to_po(po_file_path, translations):
    """Merge translations back into the main PO file."""
    with open(po_file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    updated_count = 0
    i = 0
    
    while i < len(lines):
        line = lines[i].strip()
        
        if line.startswith('msgid ') and i + 1 < len(lines):
            # Extract msgid content
            msgid_match = re.match(r'msgid "([^"]*)"', line)
            if msgid_match:
                msgid_text = msgid_match.group(1)
                
                # Check if we have a translation for this msgid
                if msgid_text in translations:
                    # Look for the next msgstr line
                    j = i + 1
                    while j < len(lines) and not lines[j].strip().startswith('msgstr '):
                        j += 1
                    
                    if j < len(lines):
                        # Update the msgstr line
                        msgstr_line = lines[j].strip()
                        if 'msgstr ""' in msgstr_line:
                            lines[j] = f'msgstr "{translations[msgid_text]}"\n'
                            updated_count += 1
        
        i += 1
    
    # Write updated content back to file
    with open(po_file_path, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    
    return updated_count

def main():
    """Merge all translated chunks."""
    chunks_dir = Path("/tmp/po_chunks")
    po_file_path = "/home/aeon/repos/AutoSDV/book/po/zh-TW.po"
    
    all_translations = {}
    translated_chunks = list(chunks_dir.glob("*_translated.po"))
    
    print(f"Found {len(translated_chunks)} translated chunks")
    
    for chunk_file in translated_chunks:
        chunk_translations = extract_translations_from_chunk(chunk_file)
        all_translations.update(chunk_translations)
        print(f"Extracted {len(chunk_translations)} translations from {chunk_file.name}")
    
    print(f"Total unique translations: {len(all_translations)}")
    
    # Merge into PO file
    updated_count = merge_translations_to_po(po_file_path, all_translations)
    print(f"Updated {updated_count} entries in PO file")

if __name__ == "__main__":
    main()