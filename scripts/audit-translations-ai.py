#!/usr/bin/env python3
"""
AI-powered semantic translation auditor using Claude CLI.

Requirements:
    - claude CLI (Claude Code) installed and configured
    - Active Claude subscription

Usage:
    # Audit all translations
    python scripts/audit-translations-ai.py

    # Audit specific file
    python scripts/audit-translations-ai.py --file src/index.md

    # Use different Claude model
    python scripts/audit-translations-ai.py --model opus

Setup:
    1. Install Claude Code: https://claude.com/code
    2. Configure subscription: claude auth
    3. Run audit: python scripts/audit-translations-ai.py

Note: Uses your configured Claude subscription. No API key needed.
"""

import os
import json
import subprocess
import hashlib
from pathlib import Path
from typing import List, Dict, Optional
import argparse
import sys


# Default Claude model (sonnet is fast and high quality)
DEFAULT_MODEL = 'sonnet'

# Cache file location
CACHE_FILE = Path(__file__).parent.parent / 'tmp' / 'audit-cache.json'


def check_claude_cli():
    """Check if claude CLI is available."""
    try:
        result = subprocess.run(
            ['claude', '--version'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            version = result.stdout.strip()
            return True, version
        return False, None
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False, None


def compute_file_hash(file_path: Path) -> str:
    """Compute SHA256 hash of file contents."""
    if not file_path.exists():
        return ""
    sha256 = hashlib.sha256()
    with open(file_path, 'rb') as f:
        sha256.update(f.read())
    return sha256.hexdigest()


def load_cache() -> Dict:
    """Load audit cache from file."""
    if not CACHE_FILE.exists():
        return {}
    try:
        with open(CACHE_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError):
        return {}


def save_cache(cache: Dict):
    """Save audit cache to file."""
    CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(CACHE_FILE, 'w', encoding='utf-8') as f:
        json.dump(cache, f, indent=2, ensure_ascii=False)


def get_cached_result(cache: Dict, en_file: Path, zh_file: Path, model: str) -> Optional[Dict]:
    """Get cached result if files haven't changed. Returns a copy without Path objects."""
    cache_key = str(en_file.relative_to(Path.cwd() / 'src'))

    if cache_key not in cache:
        return None

    cached = cache[cache_key]

    # Check if model changed
    if cached.get('model') != model:
        return None

    # Check if files changed (compare hashes)
    en_hash = compute_file_hash(en_file)
    zh_hash = compute_file_hash(zh_file) if zh_file.exists() else ""

    if cached.get('en_hash') != en_hash or cached.get('zh_hash') != zh_hash:
        return None

    # Return a copy of the result (without Path objects)
    result = cached.get('result')
    if result:
        return {
            'status': result.get('status'),
            'issues': result.get('issues', []),
            'summary': result.get('summary', '')
        }
    return None


def call_claude(prompt: str, model: str = DEFAULT_MODEL, json_output: bool = True) -> Dict:
    """Call claude CLI with a prompt and return response."""
    cmd = ['claude', '-p', '--model', model]

    if json_output:
        # Define JSON schema for structured output
        schema = {
            "type": "object",
            "properties": {
                "is_accurate": {"type": "boolean"},
                "severity": {"type": "string", "enum": ["none", "low", "medium", "high"]},
                "issues": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "type": {"type": "string"},
                            "severity": {"type": "string"},
                            "description": {"type": "string"}
                        },
                        "required": ["type", "description"]
                    }
                },
                "summary": {"type": "string"}
            },
            "required": ["is_accurate", "severity", "issues", "summary"]
        }
        cmd.extend(['--output-format', 'json', '--json-schema', json.dumps(schema)])

    cmd.append(prompt)

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60  # 60 second timeout per request
        )

        if result.returncode != 0:
            return {
                "is_accurate": None,
                "severity": "unknown",
                "issues": [],
                "summary": f"Claude CLI error: {result.stderr}",
                "error": result.stderr
            }

        if json_output:
            try:
                return json.loads(result.stdout)
            except json.JSONDecodeError as e:
                return {
                    "is_accurate": None,
                    "severity": "unknown",
                    "issues": [],
                    "summary": f"JSON parse error: {str(e)}",
                    "error": str(e)
                }
        else:
            return {"response": result.stdout}

    except subprocess.TimeoutExpired:
        return {
            "is_accurate": None,
            "severity": "unknown",
            "issues": [],
            "summary": "Request timeout",
            "error": "timeout"
        }
    except Exception as e:
        return {
            "is_accurate": None,
            "severity": "unknown",
            "issues": [],
            "summary": f"Error: {str(e)}",
            "error": str(e)
        }


def analyze_translation_pair(en_file: Path, zh_file: Path, model: str, cache: Dict = None, use_cache: bool = True) -> Dict:
    """Use Claude to analyze semantic differences between English and Chinese files."""

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

    # Check cache first
    if use_cache and cache is not None:
        cached_result = get_cached_result(cache, en_file, zh_file, model)
        if cached_result is not None:
            print(f"  Using cached result...", flush=True)
            # Restore file paths (not stored in cache)
            cached_result['en_file'] = en_file
            cached_result['zh_file'] = zh_file
            return cached_result

    try:
        en_content = en_file.read_text(encoding='utf-8')
        zh_content = zh_file.read_text(encoding='utf-8')
    except Exception as e:
        result['status'] = 'error'
        result['summary'] = f'Error reading files: {e}'
        return result

    # Analyze full document
    print(f"  Analyzing with Claude...", flush=True)

    # Limit content length for prompt
    en_preview = en_content[:3000]
    zh_preview = zh_content[:3000]

    prompt = f"""Compare this English documentation with its Chinese (Traditional) translation.

Detect semantic differences:
1. Missing sections or content in the translation
2. Structural organization differences
3. Semantic drift in key concepts
4. Outdated information (old versions, deprecated features)

English (first 3000 chars):
{en_preview}

Chinese (first 3000 chars):
{zh_preview}

Respond with JSON containing:
- is_accurate: boolean (true if translation is semantically accurate)
- severity: "none" | "low" | "medium" | "high"
- issues: array of objects with type, severity, description
- summary: brief overall assessment

Be concise and focus on significant semantic differences, not minor stylistic choices.
"""

    claude_result = call_claude(prompt, model=model, json_output=True)

    if claude_result.get('error'):
        result['issues'].append({
            'type': 'error',
            'severity': 'high',
            'description': f'Analysis error: {claude_result["summary"]}'
        })
    else:
        result['issues'] = claude_result.get('issues', [])
        result['summary'] = claude_result.get('summary', '')

    # Determine status
    if not result['issues']:
        result['status'] = 'up_to_date'
    else:
        high_severity = any(i.get('severity') == 'high' for i in result['issues'])
        result['status'] = 'semantic_drift' if high_severity else 'minor_issues'

    # Update cache with new result
    if cache is not None:
        cache_key = str(en_file.relative_to(Path.cwd() / 'src'))
        cache[cache_key] = {
            'model': model,
            'en_hash': compute_file_hash(en_file),
            'zh_hash': compute_file_hash(zh_file),
            'result': {
                'status': result['status'],
                'issues': result['issues'],
                'summary': result['summary']
            }
        }

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
        description='AI-powered semantic translation auditor using Claude CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Audit all translations (uses cache)
  python scripts/audit-translations-ai.py

  # Audit specific file
  python scripts/audit-translations-ai.py --file src/index.md

  # Use different Claude model
  python scripts/audit-translations-ai.py --model opus

  # Force re-audit (ignore cache)
  python scripts/audit-translations-ai.py --no-cache

Caching:
  - Results are cached in tmp/audit-cache.json
  - Only re-audits files that have changed (based on file hash)
  - Use --no-cache to force re-audit all files
  - Cache is automatically invalidated when model changes

Setup:
  1. Install Claude Code: https://claude.com/code
  2. Configure: claude auth
  3. Run: python scripts/audit-translations-ai.py
"""
    )

    parser.add_argument('--file', type=str,
                        help='Specific file to audit (relative to project root)')
    parser.add_argument('--model', type=str, default=DEFAULT_MODEL,
                        help=f'Claude model to use: sonnet (default), opus, haiku')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose logging')
    parser.add_argument('--no-cache', action='store_true',
                        help='Disable caching and re-audit all files')

    args = parser.parse_args()
    use_cache = not args.no_cache

    # Check if claude CLI is available
    available, version = check_claude_cli()
    if not available:
        print("Error: claude CLI not found")
        print()
        print("Setup instructions:")
        print("  1. Install Claude Code from: https://claude.com/code")
        print("  2. Authenticate: claude auth")
        print("  3. Verify: claude --version")
        return 1

    if args.verbose:
        print(f"Using Claude CLI: {version}")

    # Find file pairs
    if args.file:
        en_file = Path(__file__).parent.parent / args.file
        zh_file = en_file.with_suffix('.zh-TW.md')
        pairs = [(en_file, zh_file)]
    else:
        pairs = find_translation_pairs()

    # Load cache
    cache = load_cache() if use_cache else {}

    print("\n🤖 AI Semantic Translation Audit (Claude)")
    print("=" * 80)
    print(f"Model: {args.model}")
    print(f"Total Files: {len(pairs)}")
    if use_cache:
        print(f"Cache: Enabled (tmp/audit-cache.json)")
    else:
        print(f"Cache: Disabled (--no-cache)")
    print("=" * 80)

    # Analyze each pair
    all_results = {
        'missing': [],
        'semantic_drift': [],
        'minor_issues': [],
        'up_to_date': []
    }

    cached_count = 0
    analyzed_count = 0

    for i, (en_file, zh_file) in enumerate(pairs, 1):
        rel_path = en_file.relative_to(Path.cwd() / 'src')
        print(f"\n[{i}/{len(pairs)}] Auditing: {rel_path}")

        # Track if this result came from cache
        was_cached = False
        if use_cache and cache:
            cached_result = get_cached_result(cache, en_file, zh_file, args.model)
            was_cached = cached_result is not None

        analysis = analyze_translation_pair(en_file, zh_file, args.model, cache=cache, use_cache=use_cache)
        all_results[analysis['status']].append(analysis)

        if was_cached:
            cached_count += 1
        else:
            analyzed_count += 1

    # Print report
    print("\n\n" + "=" * 80)
    print("📊 AI Semantic Audit Results")
    print("=" * 80)

    # Missing translations
    if all_results['missing']:
        print(f"\n❌ Missing Translations ({len(all_results['missing'])}):")
        for item in all_results['missing']:
            rel_path = item['en_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}")

    # Semantic drift
    if all_results['semantic_drift']:
        print(f"\n🔴 Semantic Drift Detected ({len(all_results['semantic_drift'])}):")
        print("     (Significant content differences)")
        for item in all_results['semantic_drift']:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"\n  📄 {rel_path}")
            if item.get('summary'):
                print(f"     Summary: {item['summary']}")

            high_issues = [i for i in item['issues'] if i.get('severity') == 'high']
            medium_issues = [i for i in item['issues'] if i.get('severity') == 'medium']

            if high_issues:
                print(f"     🔴 High severity: {len(high_issues)} issues")
                for issue in high_issues[:3]:
                    print(f"        - {issue['description']}")

            if medium_issues:
                print(f"     🟡 Medium severity: {len(medium_issues)} issues")
                for issue in medium_issues[:2]:
                    print(f"        - {issue['description']}")

    # Minor issues
    if all_results['minor_issues']:
        print(f"\n🟡 Minor Issues ({len(all_results['minor_issues'])}):")
        for item in all_results['minor_issues']:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}: {len(item['issues'])} minor issues")

    # Up to date
    if all_results['up_to_date']:
        print(f"\n✅ Semantically Accurate ({len(all_results['up_to_date'])}):")
        for item in all_results['up_to_date'][:5]:
            rel_path = item['zh_file'].relative_to(Path.cwd() / 'src')
            print(f"  - {rel_path}")
        if len(all_results['up_to_date']) > 5:
            print(f"  ... and {len(all_results['up_to_date']) - 5} more")

    # Summary
    print("\n" + "=" * 80)
    print(f"Total: {len(all_results['up_to_date'])} accurate, "
          f"{len(all_results['minor_issues'])} minor issues, "
          f"{len(all_results['semantic_drift'])} semantic drift, "
          f"{len(all_results['missing'])} missing")
    print("=" * 80)

    # Save cache
    if use_cache:
        save_cache(cache)

    # Info
    print("\n💎 Using Claude CLI")
    print(f"   Model: {args.model}")
    print(f"   Total files: {len(pairs)}")
    print(f"   Analyzed: {analyzed_count} files")
    if use_cache:
        print(f"   Cached: {cached_count} files")
        print(f"   Cache location: {CACHE_FILE.relative_to(Path.cwd())}")
    if analyzed_count > 0:
        print(f"   Estimated time saved: ~{cached_count * 3} seconds")

    return 0


if __name__ == '__main__':
    exit(main())
