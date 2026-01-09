#!/usr/bin/env python3
"""
AI-powered semantic translation auditor.

Uses Claude AI to detect semantic differences between English and Chinese translations.
Goes beyond structural comparison to understand actual content meaning.

Requirements:
    pip install anthropic

Usage:
    export ANTHROPIC_API_KEY="your-api-key"
    python scripts/audit-translations-ai.py
    python scripts/audit-translations-ai.py --file src/index.md
    python scripts/audit-translations-ai.py --section-analysis
"""

import os
import re
import json
from pathlib import Path
from typing import List, Dict
import argparse

try:
    from anthropic import Anthropic
except ImportError:
    print("Error: anthropic package not installed")
    print("Install with: pip install anthropic")
    exit(1)


def extract_sections(content: str) -> List[Dict]:
    """Extract markdown sections by headings."""
    sections = []
    current_section = None

    lines = content.split('\n')
    for i, line in enumerate(lines):
        heading_match = re.match(r'^(#{1,6})\s+(.+)$', line)

        if heading_match:
            if current_section:
                sections.append(current_section)

            level = len(heading_match.group(1))
            title = heading_match.group(2).strip()
            current_section = {
                'level': level,
                'title': title,
                'content': '',
                'line_start': i + 1
            }
        elif current_section is not None:
            current_section['content'] += line + '\n'

    if current_section:
        sections.append(current_section)

    return sections


def compare_sections_with_ai(en_section: Dict, zh_section: Dict, client: Anthropic) -> Dict:
    """Use AI to compare semantic content of two sections."""

    prompt = f"""You are comparing an English documentation section with its Chinese translation to detect semantic differences.

English Section:
Title: {en_section['title']}
Content:
{en_section['content'][:2000]}

Chinese Section:
Title: {zh_section['title']}
Content:
{zh_section['content'][:2000]}

Analyze if the Chinese translation accurately reflects the English content. Check for:
1. Missing information (content in English not present in Chinese)
2. Extra information (content in Chinese not in English)
3. Semantic differences (different meaning/emphasis)
4. Outdated information (references to old versions, deprecated features)

Respond in JSON format:
{{
    "is_accurate": true/false,
    "severity": "none|low|medium|high",
    "issues": [
        {{
            "type": "missing_content|extra_content|semantic_drift|outdated",
            "description": "brief description of the issue"
        }}
    ],
    "summary": "brief summary of translation quality"
}}
"""

    try:
        message = client.messages.create(
            model="claude-3-5-haiku-20241022",
            max_tokens=1024,
            messages=[{"role": "user", "content": prompt}]
        )

        response_text = message.content[0].text
        json_match = re.search(r'```json\n(.*?)\n```', response_text, re.DOTALL)
        if json_match:
            json_text = json_match.group(1)
        else:
            json_text = response_text

        result = json.loads(json_text)
        return result

    except Exception as e:
        return {
            "is_accurate": None,
            "severity": "unknown",
            "issues": [],
            "summary": f"Error during AI analysis: {str(e)}",
            "error": str(e)
        }


def analyze_translation_pair_ai(en_file: Path, zh_file: Path, client: Anthropic,
                                section_analysis: bool = False) -> Dict:
    """Use AI to analyze semantic differences between English and Chinese files."""

    result = {
        'en_file': en_file,
        'zh_file': zh_file,
        'status': 'unknown',
        'issues': [],
        'summary': ''
    }

    if not zh_file.exists():
        result['status'] = 'missing'
        result['summary'] = 'No Chinese translation exists'
        return result

    try:
        en_content = en_file.read_text(encoding='utf-8')
        zh_content = zh_file.read_text(encoding='utf-8')
    except Exception as e:
        result['status'] = 'error'
        result['summary'] = f'Error reading files: {e}'
        return result

    if section_analysis:
        # Section-by-section analysis
        en_sections = extract_sections(en_content)
        zh_sections = extract_sections(zh_content)

        if len(en_sections) != len(zh_sections):
            result['issues'].append({
                'type': 'structure',
                'severity': 'high',
                'description': f'Section count mismatch: EN has {len(en_sections)} sections, ZH has {len(zh_sections)} sections'
            })

        for i, (en_sec, zh_sec) in enumerate(zip(en_sections, zh_sections)):
            print(f"  Analyzing section {i+1}/{min(len(en_sections), len(zh_sections))}: {en_sec['title'][:50]}...", flush=True)

            ai_result = compare_sections_with_ai(en_sec, zh_sec, client)

            if not ai_result.get('is_accurate', True):
                result['issues'].extend([
                    {
                        **issue,
                        'section': en_sec['title'],
                        'line': en_sec['line_start']
                    }
                    for issue in ai_result.get('issues', [])
                ])
    else:
        # Whole-file comparison
        print(f"  Analyzing full document with AI...", flush=True)

        en_preview = en_content[:3000]
        zh_preview = zh_content[:3000]

        prompt = f"""Compare this English documentation with its Chinese translation.

English (first 3000 chars):
{en_preview}

Chinese (first 3000 chars):
{zh_preview}

Detect major differences in:
1. Missing sections or content
2. Structural organization differences
3. Semantic drift in key concepts

Respond in JSON format:
{{
    "is_accurate": true/false,
    "severity": "none|low|medium|high",
    "issues": [
        {{
            "type": "missing_content|structure|semantic_drift",
            "severity": "low|medium|high",
            "description": "brief description"
        }}
    ],
    "summary": "overall assessment"
}}
"""

        try:
            message = client.messages.create(
                model="claude-3-5-haiku-20241022",
                max_tokens=1024,
                messages=[{"role": "user", "content": prompt}]
            )

            response_text = message.content[0].text
            json_match = re.search(r'```json\n(.*?)\n```', response_text, re.DOTALL)
            if json_match:
                json_text = json_match.group(1)
            else:
                json_text = response_text

            ai_result = json.loads(json_text)
            result['issues'] = ai_result.get('issues', [])
            result['summary'] = ai_result.get('summary', '')

        except Exception as e:
            result['issues'].append({
                'type': 'error',
                'severity': 'high',
                'description': f'AI analysis error: {str(e)}'
            })

    # Determine status
    if not result['issues']:
        result['status'] = 'up_to_date'
    else:
        high_severity = any(i.get('severity') == 'high' for i in result['issues'])
        result['status'] = 'semantic_drift' if high_severity else 'minor_issues'

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
    parser = argparse.ArgumentParser(description='AI-powered semantic translation auditor')
    parser.add_argument('--file', type=str, help='Audit specific file')
    parser.add_argument('--section-analysis', action='store_true',
                        help='Detailed section-by-section analysis (slower but thorough)')
    parser.add_argument('--api-key', type=str,
                        help='Anthropic API key (or set ANTHROPIC_API_KEY env var)')
    args = parser.parse_args()

    # Get API key
    api_key = args.api_key or os.environ.get('ANTHROPIC_API_KEY')
    if not api_key:
        print("Error: ANTHROPIC_API_KEY not set")
        print("\nUsage:")
        print("  export ANTHROPIC_API_KEY='your-api-key'")
        print("  python scripts/audit-translations-ai.py")
        print("\nOr:")
        print("  python scripts/audit-translations-ai.py --api-key 'your-api-key'")
        print("\nGet your API key from: https://console.anthropic.com/")
        return 1

    client = Anthropic(api_key=api_key)

    # Find file pairs
    if args.file:
        en_file = Path(__file__).parent.parent / args.file
        zh_file = en_file.with_suffix('.zh-TW.md')
        pairs = [(en_file, zh_file)]
    else:
        pairs = find_translation_pairs()

    print("\n🤖 AI Semantic Translation Audit")
    print("=" * 80)
    print(f"Model: Claude 3.5 Haiku (fast, cost-effective)")
    print(f"Mode: {'Section-by-section' if args.section_analysis else 'Whole-file'}")
    print(f"Files: {len(pairs)}")
    print("=" * 80)

    # Analyze each pair
    results = {
        'missing': [],
        'semantic_drift': [],
        'minor_issues': [],
        'up_to_date': []
    }

    for i, (en_file, zh_file) in enumerate(pairs, 1):
        rel_path = en_file.relative_to(Path.cwd() / 'src')
        print(f"\n[{i}/{len(pairs)}] Auditing: {rel_path}")

        analysis = analyze_translation_pair_ai(
            en_file, zh_file, client,
            section_analysis=args.section_analysis
        )

        results[analysis['status']].append(analysis)

    # Print report
    print("\n\n" + "=" * 80)
    print("📊 AI Semantic Audit Results")
    print("=" * 80)

    # Missing translations
    if results['missing']:
        print(f"\n❌ Missing Translations ({len(results['missing'])}):")
        for item in results['missing']:
            rel_path = item['en_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}")

    # Semantic drift
    if results['semantic_drift']:
        print(f"\n🔴 Semantic Drift Detected ({len(results['semantic_drift'])}):")
        print("     (Significant content differences)")
        for item in results['semantic_drift']:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"\n  📄 {rel_path}")
            if item.get('summary'):
                print(f"     Summary: {item['summary']}")

            high_issues = [i for i in item['issues'] if i.get('severity') == 'high']
            medium_issues = [i for i in item['issues'] if i.get('severity') == 'medium']

            if high_issues:
                print(f"     🔴 High severity: {len(high_issues)} issues")
                for issue in high_issues[:3]:
                    section = issue.get('section', '')
                    section_str = f" (in section: {section})" if section else ""
                    print(f"        - {issue['description']}{section_str}")

            if medium_issues:
                print(f"     🟡 Medium severity: {len(medium_issues)} issues")
                for issue in medium_issues[:2]:
                    section = issue.get('section', '')
                    section_str = f" (in section: {section})" if section else ""
                    print(f"        - {issue['description']}{section_str}")

    # Minor issues
    if results['minor_issues']:
        print(f"\n🟡 Minor Issues ({len(results['minor_issues'])}):")
        for item in results['minor_issues']:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}: {len(item['issues'])} minor issues")

    # Up to date
    if results['up_to_date']:
        print(f"\n✅ Semantically Accurate ({len(results['up_to_date'])}):")
        for item in results['up_to_date'][:5]:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}")
        if len(results['up_to_date']) > 5:
            print(f"  ... and {len(results['up_to_date']) - 5} more")

    # Summary
    print("\n" + "=" * 80)
    print(f"Total: {len(results['up_to_date'])} accurate, "
          f"{len(results['minor_issues'])} minor issues, "
          f"{len(results['semantic_drift'])} semantic drift, "
          f"{len(results['missing'])} missing")
    print("=" * 80)

    # Cost estimate
    total_checks = len([r for r in results.values() for _ in r if _ not in results['missing']])
    estimated_cost = total_checks * 0.001
    print(f"\nEstimated API cost: ~${estimated_cost:.3f} USD")

    # Exit with error if issues exist
    if results['missing'] or results['semantic_drift']:
        return 1
    return 0


if __name__ == '__main__':
    exit(main())
