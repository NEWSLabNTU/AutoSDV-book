default:
	@just --list

setup:
	@echo "Installing MkDocs and dependencies..."
	@command -v pip3 >/dev/null 2>&1 || (echo "Error: pip3 not found. Please install Python 3 and pip3." && exit 1)
	@pip3 install -r requirements.txt
	@echo ""
	@echo "[OK] Setup complete!"
	@echo "  Run 'just build' to build the documentation"
	@echo "  Run 'just serve' to serve the documentation"

build:
	@echo "Building all language versions..."
	@mkdocs build
	@echo ""
	@echo "[OK] Build complete!"
	@echo "  Output: site/"
	@echo "  English:  site/en/"
	@echo "  Chinese:  site/zh-TW/"
	@echo ""
	@echo "  Open site/index.html to view the documentation"

build-en:
	@echo "Building English version only..."
	@mkdocs build -f mkdocs.yml
	@echo ""
	@echo "[OK] Build complete!"
	@echo "  Output: site/"

build-zh:
	@echo "Building Chinese version only..."
	@mkdocs build -f mkdocs.yml
	@echo ""
	@echo "[OK] Build complete!"
	@echo "  Output: site/"

serve:
	@echo ""
	@echo "======================================================"
	@echo "  AutoSDV Book Server Running"
	@echo "======================================================"
	@echo ""
	@echo "  Documentation:      http://localhost:8000"
	@echo "  English:            http://localhost:8000/en/"
	@echo "  Traditional Chinese: http://localhost:8000/zh-TW/"
	@echo ""
	@echo "  Press Ctrl+C to stop the server"
	@echo "======================================================"
	@echo ""
	@mkdocs serve -a 0.0.0.0:3000

clean:
	@echo "Cleaning build outputs..."
	@rm -rf site
	@echo "[OK] Clean complete!"

# Development helpers
watch:
	@echo "Watching for changes and rebuilding..."
	@mkdocs serve --watch src

deploy:
	@echo "Deploying to GitHub Pages..."
	@mkdocs gh-deploy --force
	@echo "[OK] Deployment complete!"

# Check if dependencies are installed
check:
	@echo "Checking MkDocs installation..."
	@command -v mkdocs >/dev/null 2>&1 && echo "[OK] mkdocs found" || echo "[X] mkdocs not found (run 'just setup')"
	@python3 -c "import mkdocs_material" 2>/dev/null && echo "[OK] mkdocs-material found" || echo "[X] mkdocs-material not found (run 'just setup')"
	@python3 -c "import mkdocs_awesome_pages_plugin" 2>/dev/null && echo "[OK] mkdocs-awesome-pages-plugin found" || echo "[X] mkdocs-awesome-pages-plugin not found (run 'just setup')"
	@python3 -c "import mkdocs_static_i18n" 2>/dev/null && echo "[OK] mkdocs-static-i18n found" || echo "[X] mkdocs-static-i18n not found (run 'just setup')"

# Lint: Validate documentation format and check translations
lint:
	@echo "========================================================================"
	@echo "Documentation Linting"
	@echo "========================================================================"
	@echo ""
	@echo "[1/2] Checking MkDocs configuration..."
	@mkdocs build --strict --quiet && echo "   [OK] MkDocs build successful (strict mode)" || (echo "   [X] MkDocs build failed" && exit 1)
	@echo ""
	@echo "[2/2] Checking translation status..."
	@python3 scripts/check-translations.py || (echo "" && echo "   [!] Translation issues found (see above)" && echo "   Run with --verbose for details: python3 scripts/check-translations.py --verbose")
	@echo ""
	@echo "========================================================================"
	@echo "[OK] Lint complete!"
	@echo "========================================================================"
	@echo ""
	@echo "Tips:"
	@echo "   - Check specific file: python3 scripts/check-translations.py --file src/index.md"
	@echo "   - Show git diff: python3 scripts/check-translations.py --show-diff"
	@echo "   - AI semantic audit: python3 scripts/audit-translations-ai.py"

# AI-based semantic translation audit using Claude CLI
audit-translations:
	#!/usr/bin/env bash
	set -euo pipefail
	echo "========================================================================"
	echo "AI-Based Semantic Translation Audit (Claude)"
	echo "========================================================================"
	echo ""
	echo "Running AI semantic analysis with Claude..."
	echo "Note: Uses your configured Claude subscription"
	echo ""
	python3 scripts/audit-translations-ai.py
	echo ""
	echo "========================================================================"
	echo "[OK] Audit complete!"
	echo "========================================================================"
	echo ""
	echo "Tips:"
	echo "   - Use different model: python3 scripts/audit-translations-ai.py --model opus"
	echo "   - Check specific file: python3 scripts/audit-translations-ai.py --file src/index.md"
	echo "   - Enable verbose mode: python3 scripts/audit-translations-ai.py --verbose"
