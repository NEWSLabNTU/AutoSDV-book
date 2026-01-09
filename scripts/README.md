# Documentation Scripts

## Overview

**check-translations.py** - Fast structural & timestamp checking
**audit-translations-ai.py** - AI semantic analysis (requires API key)

---

## check-translations.py

Detects missing translations, outdated files, and structural differences.

### Usage

```bash
# Check all files
python3 scripts/check-translations.py

# Check specific file
python3 scripts/check-translations.py --file src/index.md

# Show details
python3 scripts/check-translations.py --verbose

# Show git diff
python3 scripts/check-translations.py --show-diff

# Via make (includes MkDocs validation)
make lint
```

### Detects

- Missing `.zh-TW.md` files
- Outdated timestamps (EN modified after ZH)
- Heading count/level mismatches
- Code block differences
- Git changes since translation

**Performance**: <5s | **Cost**: Free | **Offline**: Yes

---

## audit-translations-ai.py

AI-powered semantic difference detection using Claude 3.5 Haiku.

### Setup

```bash
pip install anthropic
export ANTHROPIC_API_KEY="your-key"  # Get from console.anthropic.com
```

### Usage

```bash
# Audit all files
python3 scripts/audit-translations-ai.py

# Audit specific file
python3 scripts/audit-translations-ai.py --file src/index.md

# Section-by-section analysis
python3 scripts/audit-translations-ai.py --section-analysis
```

### Detects

- All from check-translations.py, plus:
- Semantic drift (meaning differences)
- Missing content (even if structure matches)
- Outdated references (SDK versions, etc.)
- Extra content in translations

**Performance**: 60-150s | **Cost**: ~$0.001/file | **Offline**: No

---

## Workflows

### Daily
```bash
make lint
```

### After updating English docs
```bash
python3 scripts/check-translations.py --file src/guides/lidar.md --show-diff
vim src/guides/lidar.zh-TW.md
```

### Pre-release
```bash
python3 scripts/audit-translations-ai.py --section-analysis
```

---

## Exit Codes

- `0` - Success
- `1` - Issues found

Suitable for CI/CD pipelines.
