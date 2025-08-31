#!/usr/bin/env python3
"""
Fix PO file formatting issues - specifically newline mismatches between msgid and msgstr.
"""

import re
import sys
from pathlib import Path

def fix_po_formatting(po_file_path):
    """Fix newline formatting issues in PO file."""
    po_file = Path(po_file_path)
    if not po_file.exists():
        print(f"Error: PO file not found: {po_file_path}")
        return False
    
    # Read the entire PO file line by line
    with open(po_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    fixed_count = 0
    i = 0
    
    while i < len(lines):
        line = lines[i].strip()
        
        # Look for msgid lines
        if line.startswith('msgid '):
            msgid_line = i
            msgstr_line = None
            
            # Find the corresponding msgstr
            j = i + 1
            while j < len(lines) and not lines[j].strip().startswith('msgstr '):
                j += 1
            
            if j < len(lines):
                msgstr_line = j
                
                # Extract the actual content from quotes
                msgid_content = lines[msgid_line].strip()
                msgstr_content = lines[msgstr_line].strip()
                
                # Check if both have quoted content
                if '"' in msgid_content and '"' in msgstr_content:
                    # Extract content between quotes
                    msgid_match = re.search(r'"([^"]*)"', msgid_content)
                    msgstr_match = re.search(r'"([^"]*)"', msgstr_content)
                    
                    if msgid_match and msgstr_match:
                        msgid_text = msgid_match.group(1)
                        msgstr_text = msgstr_match.group(1)
                        
                        # Check newline ending
                        msgid_ends_newline = msgid_text.endswith('\\n')
                        msgstr_ends_newline = msgstr_text.endswith('\\n')
                        
                        if msgid_ends_newline != msgstr_ends_newline:
                            if msgid_ends_newline and not msgstr_ends_newline and msgstr_text.strip():
                                # Add \n to msgstr
                                new_msgstr = msgstr_text + '\\n'
                                lines[msgstr_line] = lines[msgstr_line].replace(f'"{msgstr_text}"', f'"{new_msgstr}"')
                                fixed_count += 1
                            elif not msgid_ends_newline and msgstr_ends_newline:
                                # Remove \n from msgstr  
                                new_msgstr = msgstr_text.rstrip('\\n')
                                lines[msgstr_line] = lines[msgstr_line].replace(f'"{msgstr_text}"', f'"{new_msgstr}"')
                                fixed_count += 1
        
        i += 1
    
    # Write the fixed content back
    with open(po_file, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    
    print(f"Fixed {fixed_count} newline formatting issues in {po_file.name}")
    return True

if __name__ == "__main__":
    po_file_path = "/home/aeon/repos/AutoSDV/book/po/zh-TW.po"
    if len(sys.argv) > 1:
        po_file_path = sys.argv[1]
    
    fix_po_formatting(po_file_path)