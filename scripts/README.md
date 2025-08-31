# Translation Workflow Scripts

This directory contains scripts for the **manual AI-based translation workflow** for the AutoSDV book.

## Workflow Overview

The translation process uses a **context-aware manual approach** where AI processes chunks one-by-one, NOT using search-and-replace automation.

### Key Principles

1. **Context-Aware Translation**: AI reads each chunk and translates with full context understanding
2. **Manual Processing**: Each chunk is processed individually by AI, not automated
3. **Quality Focus**: Taiwanese Traditional Chinese with proper terminology preservation
4. **Code Preservation**: All code, scripts, and commands remain untranslated (only comments are translated)

## Scripts

### 1. `translate_workflow.sh`
Main orchestrator script for the translation workflow.

**Common Commands:**
```bash
./translate_workflow.sh start    # Reset and split PO file into chunks
./translate_workflow.sh status   # Check translation progress
./translate_workflow.sh merge    # Merge translated chunks back
./translate_workflow.sh build    # Build the book with translations
./translate_workflow.sh finish   # Merge and build in one step
```

### 2. `split_for_manual_translation.py`
Splits the PO file into manageable chunks for manual AI translation.

**Usage:**
```bash
python3 split_for_manual_translation.py reset  # Reset and split
```

- Creates chunks of 20 entries each in `/tmp/po_chunks/`
- Extracts only untranslated entries
- Preserves source file references for context

### 3. `merge_manual_translations.py`
Merges manually translated chunks back into the main PO file.

**Usage:**
```bash
python3 merge_manual_translations.py
```

- Collects all `*_translated.po` files from `/tmp/po_chunks/`
- Updates the main `po/zh-TW.po` file
- Preserves existing translations

### 4. `fix_po_formatting.py`
Fixes common PO file formatting issues.

**Usage:**
```bash
python3 fix_po_formatting.py [po_file]
```

- Fixes newline mismatches between msgid and msgstr
- Ensures proper PO format compliance

## Translation Process

### Step 1: Initialize
```bash
./scripts/translate_workflow.sh start
```
This will:
- Reset all translations in the PO file
- Split untranslated entries into chunks in `/tmp/po_chunks/`

### Step 2: Manual AI Translation
AI processes each chunk file manually:
1. Read chunk file (e.g., `chunk_001.po`)
2. Understand context from source references
3. Create translated version (`chunk_001_translated.po`)
4. Apply quality guidelines:
   - Use Taiwanese Traditional Chinese
   - Keep technical terms in English when appropriate
   - Preserve all code/commands unchanged
   - Translate only comments within code

### Step 3: Merge Results
```bash
./scripts/translate_workflow.sh merge
```
Collects all translated chunks and updates the main PO file.

### Step 4: Build and Validate
```bash
./scripts/translate_workflow.sh build
```
Builds the book and validates translations.

## Translation Guidelines

### Do Translate:
- UI text and navigation elements
- Descriptions and explanations
- Comments within code blocks
- Error messages and warnings

### Do NOT Translate:
- Code snippets
- Commands and file paths
- Configuration values
- Technical specifications (e.g., "100m", "32-bit")
- Product names (e.g., "Velodyne VLP-32C")

### Quality Standards:
- **Language**: Taiwanese Traditional Chinese (zh-TW)
- **Technical Terms**: Keep in English unless well-established Chinese term exists
- **Consistency**: Maintain consistent terminology throughout
- **Context**: Use source file references to understand context

## Chunk Files Structure

Each chunk file contains:
```po
#: src/file.md:line_number
msgid "Original English text"
msgstr ""  # To be filled with translation
```

Translated chunk:
```po
#: src/file.md:line_number
msgid "Original English text"
msgstr "繁體中文翻譯"
```

## Notes

- Chunks are stored in `/tmp/po_chunks/` (temporary directory)
- Default chunk size is 20 entries for optimal AI processing
- The workflow supports incremental translation
- Always validate PO file after merging (`msgfmt --check`)