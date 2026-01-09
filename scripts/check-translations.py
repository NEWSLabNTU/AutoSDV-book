#!/usr/bin/env python3
"""
Check translation files for missing translations and structural differences.

Combines timestamp-based checking with structural analysis to detect:
- Missing translation files
- Outdated translations (timestamp-based)
- Structural differences (headings, code blocks)
- Section-level issues

Usage:
    python scripts/check-translations.py
    python scripts/check-translations.py --file src/index.md
    python scripts/check-translations.py --verbose
    python scripts/check-translations.py --show-diff
"""

import os
import re
import subprocess
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Tuple
import argparse


def get_git_last_modified(file_path):
    """Get the last commit date for a file."""
    try:
        result = subprocess.run(
            ['git', 'log', '-1', '--format=%ai', '--', str(file_path)],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent
        )
        if result.returncode == 0 and result.stdout.strip():
            return datetime.fromisoformat(result.stdout.strip().split()[0])
    except Exception:
        pass
    return None


def get_git_commit_hash(file_path):
    """Get the last commit hash for a file."""
    try:
        result = subprocess.run(
            ['git', 'log', '-1', '--format=%h', '--', str(file_path)],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return None


def get_git_diff_since_translation(en_file, zh_date):
    """Get git diff of English file since the translation was last updated."""
    try:
        since_date = zh_date.strftime('%Y-%m-%d')
        result = subprocess.run(
            ['git', 'log', f'--since={since_date}', '--oneline', '--', str(en_file)],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent.parent
        )

        if result.returncode == 0 and result.stdout.strip():
            commits = result.stdout.strip().split('\n')
            if commits:
                oldest_commit = commits[-1].split()[0]
                diff_result = subprocess.run(
                    ['git', 'diff', f'{oldest_commit}^', 'HEAD', '--', str(en_file)],
                    capture_output=True,
                    text=True,
                    cwd=Path(__file__).parent.parent
                )
                if diff_result.returncode == 0:
                    return diff_result.stdout
    except Exception:
        pass
    return None


def extract_headings(file_path):
    """Extract markdown headings from a file."""
    if not file_path.exists():
        return []

    try:
        content = file_path.read_text(encoding='utf-8')
        heading_pattern = r'^(#{1,6})\s+(.+)$'
        headings = []

        for line in content.split('\n'):
            match = re.match(heading_pattern, line)
            if match:
                level = len(match.group(1))
                text = match.group(2).strip()
                headings.append((level, text))

        return headings
    except Exception:
        return []


def extract_code_blocks(file_path):
    """Extract code blocks from a file."""
    if not file_path.exists():
        return []

    try:
        content = file_path.read_text(encoding='utf-8')
        code_pattern = r'```[\w]*\n(.*?)```'
        code_blocks = re.findall(code_pattern, content, re.DOTALL)
        return code_blocks
    except Exception:
        return []


def compare_structure(en_headings, zh_headings):
    """Compare heading structures between English and Chinese files."""
    issues = []

    en_structure = [level for level, _ in en_headings]
    zh_structure = [level for level, _ in zh_headings]

    if len(en_structure) != len(zh_structure):
        issues.append({
            'type': 'structure_mismatch',
            'severity': 'high',
            'message': f'Heading count mismatch: EN has {len(en_structure)} headings, ZH has {len(zh_structure)} headings'
        })

    for i, (en_level, zh_level) in enumerate(zip(en_structure, zh_structure)):
        if en_level != zh_level:
            issues.append({
                'type': 'level_mismatch',
                'severity': 'medium',
                'message': f'Heading {i+1}: Level mismatch (EN: h{en_level}, ZH: h{zh_level})',
                'line': i + 1
            })

    return issues


def compare_code_blocks(en_code, zh_code):
    """Compare code blocks to ensure technical content matches."""
    issues = []

    if len(en_code) != len(zh_code):
        issues.append({
            'type': 'code_block_count',
            'severity': 'high',
            'message': f'Code block count mismatch: EN has {len(en_code)}, ZH has {len(zh_code)}'
        })

    for i, (en_block, zh_block) in enumerate(zip(en_code, zh_code)):
        en_normalized = '\n'.join(line.rstrip() for line in en_block.split('\n'))
        zh_normalized = '\n'.join(line.rstrip() for line in zh_block.split('\n'))

        if en_normalized != zh_normalized:
            issues.append({
                'type': 'code_block_diff',
                'severity': 'high',
                'message': f'Code block {i+1} differs between EN and ZH',
                'block_num': i + 1
            })

    return issues


def analyze_translation_pair(en_file, zh_file, show_diff=False):
    """Analyze a single English-Chinese file pair."""
    result = {
        'en_file': en_file,
        'zh_file': zh_file,
        'status': 'unknown',
        'issues': [],
        'en_date': None,
        'zh_date': None,
        'diff': None
    }

    if not zh_file.exists():
        result['status'] = 'missing'
        return result

    en_date = get_git_last_modified(en_file)
    zh_date = get_git_last_modified(zh_file)
    result['en_date'] = en_date
    result['zh_date'] = zh_date

    # Extract and compare structure
    en_headings = extract_headings(en_file)
    zh_headings = extract_headings(zh_file)
    structure_issues = compare_structure(en_headings, zh_headings)
    result['issues'].extend(structure_issues)

    # Extract and compare code blocks
    en_code = extract_code_blocks(en_file)
    zh_code = extract_code_blocks(zh_file)
    code_issues = compare_code_blocks(en_code, zh_code)
    result['issues'].extend(code_issues)

    # Check timestamp
    if en_date and zh_date and en_date > zh_date:
        result['status'] = 'outdated'
        result['issues'].append({
            'type': 'timestamp',
            'severity': 'high',
            'message': f'English modified {(en_date - zh_date).days} days after translation'
        })

        if show_diff:
            diff = get_git_diff_since_translation(en_file, zh_date)
            if diff:
                result['diff'] = diff
    elif result['issues']:
        result['status'] = 'structural_issues'
    else:
        result['status'] = 'up_to_date'

    return result


def find_translation_pairs():
    """Find all English-Chinese file pairs."""
    src_dir = Path(__file__).parent.parent / 'src'
    pairs = []

    for en_file in src_dir.rglob('*.md'):
        if '.zh-TW' not in en_file.name:
            zh_file = en_file.with_suffix('.zh-TW.md')
            pairs.append((en_file, zh_file))

    return pairs


def main():
    parser = argparse.ArgumentParser(
        description='Check translation files for missing and structural differences'
    )
    parser.add_argument('--file', type=str, help='Check specific file')
    parser.add_argument('--show-diff', action='store_true',
                        help='Show git diff for outdated translations')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed analysis')
    args = parser.parse_args()

    # Find file pairs
    if args.file:
        en_file = Path(__file__).parent.parent / args.file
        zh_file = en_file.with_suffix('.zh-TW.md')
        pairs = [(en_file, zh_file)]
    else:
        pairs = find_translation_pairs()

    # Analyze each pair
    results = {
        'missing': [],
        'outdated': [],
        'structural_issues': [],
        'up_to_date': []
    }

    for en_file, zh_file in pairs:
        analysis = analyze_translation_pair(en_file, zh_file, args.show_diff)
        results[analysis['status']].append(analysis)

    # Print report
    print("\n📊 Translation Status Report")
    print("=" * 80)

    # Missing translations
    if results['missing']:
        print(f"\n❌ Missing Translations ({len(results['missing'])}):")
        for item in results['missing']:
            rel_path = item['en_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}")

    # Structural issues
    if results['structural_issues']:
        print(f"\n⚠️  Structural Issues ({len(results['structural_issues'])}):")
        print("     (Translation exists but structure differs)")
        for item in results['structural_issues']:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"\n  📄 {rel_path}")

            high = [i for i in item['issues'] if i['severity'] == 'high']
            medium = [i for i in item['issues'] if i['severity'] == 'medium']

            if high:
                print(f"     🔴 High priority: {len(high)} issues")
                for issue in high:
                    print(f"        - {issue['message']}")

            if medium and args.verbose:
                print(f"     🟡 Medium priority: {len(medium)} issues")
                for issue in medium:
                    print(f"        - {issue['message']}")

    # Outdated
    if results['outdated']:
        print(f"\n⏰ Outdated Translations ({len(results['outdated'])}):")
        print("     (English source modified after translation)")
        for item in results['outdated']:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            days_old = (item['en_date'] - item['zh_date']).days
            print(f"\n  📄 {rel_path}")
            print(f"     Modified: {days_old} days after translation")
            print(f"     English: {item['en_date'].date()}")
            print(f"     Chinese: {item['zh_date'].date()}")

            non_timestamp_issues = [i for i in item['issues'] if i['type'] != 'timestamp']
            if non_timestamp_issues:
                print(f"     Structural issues: {len(non_timestamp_issues)}")
                for issue in non_timestamp_issues[:3]:
                    print(f"        - {issue['message']}")

            if args.show_diff and item.get('diff'):
                print(f"\n     Git diff since translation:")
                print("     " + "-" * 70)
                diff_lines = item['diff'].split('\n')
                for line in diff_lines[:20]:
                    print(f"     {line}")
                if len(diff_lines) > 20:
                    remaining = len(diff_lines) - 20
                    print(f"     ... ({remaining} more lines)")
                print("     " + "-" * 70)

    # Up to date
    if results['up_to_date']:
        print(f"\n✅ Up to Date ({len(results['up_to_date'])}):")
        for item in results['up_to_date'][:5]:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}")
        if len(results['up_to_date']) > 5:
            print(f"  ... and {len(results['up_to_date']) - 5} more")

    # Summary
    print("\n" + "=" * 80)
    total = len(pairs)
    print(f"Total: {len(results['up_to_date'])} up to date, "
          f"{len(results['structural_issues'])} structural issues, "
          f"{len(results['outdated'])} outdated, "
          f"{len(results['missing'])} missing")
    print("=" * 80)

    # Exit with error if issues exist
    if results['missing'] or results['outdated'] or results['structural_issues']:
        return 1
    return 0


if __name__ == '__main__':
    exit(main())
