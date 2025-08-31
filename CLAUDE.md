# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands
- Build: `make build` - Compiles the HTML document using mdbook (both English and Chinese versions)
- Serve: `make serve` - Starts a web server at http://127.0.0.1:3000 with the compiled book
- Clean: `make clean` - Removes the compiled HTML document
- Extract Messages: `make extract-messages` - Extracts translatable messages to PO file

## GitHub Actions
- The repository uses GitHub Actions to automatically deploy the book to GitHub Pages
- Deployments are triggered on pushes to `main` or `2025.02` branches
- The deploy workflow also updates the NEWSLabNTU.github.io website repository's submodule
- The workflow is defined in `.github/workflows/deploy.yml`
- A deploy key named `AUTOSDV_BOOK_DEPLOY_KEY` must be added to repository secrets
- No manual deployment is needed - just push changes to the repository

## Style Guidelines
- Use consistent Markdown formatting in `.md` files
- Include alt text for all images in the `src/figures` directory
- Organize new content in appropriate sections in the `SUMMARY.md` file
- Use relative links for internal references between pages
- Maintain a clear, instructional tone consistent with existing documentation
- For code examples, use proper syntax highlighting with triple backticks and language identifier
- Keep file and directory names lowercase with underscores for spaces
- Ensure all images are properly sized and optimized for web viewing

## Project Structure
- Content files should be placed in the `src/` directory
- Images should be placed in the `src/figures/` directory
- Update `src/SUMMARY.md` when adding new content files
- Translation files are in `po/` directory (zh-TW.po for Traditional Chinese)
- Translation scripts are in `scripts/` directory
- Language-specific config: `book-zh-TW.toml` for Chinese version

## Translation Workflow

The book supports Traditional Chinese (zh-TW) translation using a **manual AI-based workflow** with context-aware translation.

### Translation Commands
```bash
# Start new translation session (reset and split)
./scripts/translate_workflow.sh start

# Check translation progress
./scripts/translate_workflow.sh status

# Merge translated chunks back
./scripts/translate_workflow.sh merge

# Build the book with translations
./scripts/translate_workflow.sh build

# Clean up translation workspace
./scripts/translate_workflow.sh clean
```

### Translation Process
1. **Split**: PO file is split into 20-entry chunks in `/tmp/po_chunks/`
2. **Manual Translation**: AI translates each chunk with context awareness
3. **Merge**: Translated chunks are merged back into `po/zh-TW.po`
4. **Build**: Book is built with both English and Chinese versions

### Translation Guidelines
**DO Translate:**
- UI text, navigation elements, descriptions
- Error messages and warnings
- Comments within code blocks (only comments, not the code itself)

**DO NOT Translate:**
- Code snippets, commands, file paths
- Configuration values, technical specifications
- Product names (e.g., Velodyne VLP-32C, NVIDIA Jetson)
- Version numbers (e.g., Ubuntu 22.04, ROS 2 Humble)

**Quality Standards:**
- Use Taiwanese Traditional Chinese (繁體中文)
- Keep technical terms in English unless well-established Chinese term exists
- Maintain consistent terminology throughout
- Examples of consistent terms:
  - 感測器 (sensors)
  - 驅動程式 (drivers)
  - 車輛介面 (vehicle interface)
  - 映像檔 (image file)
  - 套件 (package)
  - 環景光達 (360° LiDAR)
  - 固態光達 (solid-state LiDAR)
  - 原始碼說明 (source code walkthrough)
  - 配線 (wiring)
  - 最佳化 (optimization)

**Taiwan-Specific Guidelines:**
- Use 固態光達型 instead of just 固態式 for clarity
- Use 環景光達 for 360° LiDAR instead of 360° 光達
- Translate Figure N. as 圖 N.
- Never use simplified Chinese characters (e.g., use 燒錄 not 烧錄)
- Keep parameter names in English (e.g., vehicle_model, not 車輛_模型)
- Don't translate file paths (e.g., keep AutoSDV/src/vehicle/ as is)
- Use Taiwan terminology (e.g., 程式 not 程序, 檔案 not 文件 for files)

### Translation Scripts
- `translate_workflow.sh` - Main workflow orchestrator
- `split_for_manual_translation.py` - Splits PO file into chunks
- `merge_manual_translations.py` - Merges translated chunks
- `fix_po_formatting.py` - Fixes PO file formatting issues
- `fix_newline_mismatches.py` - Fixes newline mismatches between msgid and msgstr
- `comprehensive_retranslate.py` - Fixes common translation issues and typos
- `fix_partial_translations.py` - Fixes search-and-replace style partial translations
- `comprehensive_fix.py` - Applies comprehensive fixes to translations
- `taiwan_localization_fix.py` - Applies Taiwan-specific localization fixes

### Configuration
- Main book config: `book.toml` (English)
- Chinese book config: `book-zh-TW.toml`
  - Sets `language = "zh-TW"`
  - Sets `title = "AutoSDV 使用手冊"`
  - References `po-file = "po/zh-TW.po"`

### Important Notes
- Chunks are temporary and stored in `/tmp/po_chunks/`
- Always validate PO file after merging: `msgfmt --check po/zh-TW.po`
- The workflow uses context-aware translation, NOT search-and-replace
- Each chunk should be translated manually with understanding of context

### Common Translation Issues and Fixes
When reviewing translations, check for:
1. **Partial translations**: Mixed English/Chinese in sentences
2. **Typos**: e.g., 產產 should be 生產
3. **Simplified Chinese**: Should be Traditional Chinese only
4. **Path translations**: File paths should remain in English
5. **Parameter translations**: Launch parameters should stay in English
6. **Newline mismatches**: msgid and msgstr should have matching newlines

To fix issues comprehensively:
```bash
# Fix Taiwan-specific localization issues
python3 scripts/taiwan_localization_fix.py

# Fix newline mismatches if validation fails
python3 scripts/fix_newline_mismatches.py po/zh-TW.po

# Validate the PO file
msgfmt --check po/zh-TW.po

# Rebuild the book
./scripts/translate_workflow.sh build
```