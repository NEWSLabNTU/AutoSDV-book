default:
	@just --list

setup:
	@echo "Installing MkDocs and dependencies..."
	@command -v uv >/dev/null 2>&1 || (echo "Error: uv not found. Please install uv: https://docs.astral.sh/uv/getting-started/installation/" && exit 1)
	@uv sync
	@echo ""
	@echo "[OK] Setup complete!"
	@echo "  Run 'just build' to build the documentation"
	@echo "  Run 'just serve' to serve the documentation"

build:
	@echo "Building all language versions..."
	@uv run mkdocs build
	@echo ""
	@echo "[OK] Build complete!"
	@echo "  Output: site/"
	@echo "  English:  site/en/"
	@echo "  Chinese:  site/zh-TW/"
	@echo ""
	@echo "  Open site/index.html to view the documentation"

build-en:
	@echo "Building English version only..."
	@uv run mkdocs build -f mkdocs.yml
	@echo ""
	@echo "[OK] Build complete!"
	@echo "  Output: site/"

build-zh:
	@echo "Building Chinese version only..."
	@uv run mkdocs build -f mkdocs.yml
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
	@uv run mkdocs serve -a 0.0.0.0:3000

clean:
	@echo "Cleaning build outputs..."
	@rm -rf site
	@echo "[OK] Clean complete!"

# Development helpers
watch:
	@echo "Watching for changes and rebuilding..."
	@uv run mkdocs serve --watch src

deploy:
	@echo "Deploying to GitHub Pages..."
	@uv run mkdocs gh-deploy --force
	@echo "[OK] Deployment complete!"

# Check if dependencies are installed
check:
	@echo "Checking MkDocs installation..."
	@uv run python -c "import mkdocs; print('[OK] mkdocs found')" 2>/dev/null || echo "[X] mkdocs not found (run 'just setup')"
	@uv run python -c "import mkdocs_material; print('[OK] mkdocs-material found')" 2>/dev/null || echo "[X] mkdocs-material not found (run 'just setup')"
	@uv run python -c "import mkdocs_awesome_pages_plugin; print('[OK] mkdocs-awesome-pages-plugin found')" 2>/dev/null || echo "[X] mkdocs-awesome-pages-plugin not found (run 'just setup')"
	@uv run python -c "import mkdocs_static_i18n; print('[OK] mkdocs-static-i18n found')" 2>/dev/null || echo "[X] mkdocs-static-i18n not found (run 'just setup')"

# Lint: Validate documentation format and check translations
lint:
	@echo "========================================================================"
	@echo "Documentation Linting"
	@echo "========================================================================"
	@echo ""
	@echo "[1/2] Checking MkDocs configuration..."
	@uv run mkdocs build --strict --quiet && echo "   [OK] MkDocs build successful (strict mode)" || (echo "   [X] MkDocs build failed" && exit 1)
	@echo ""
	@echo "[2/2] Checking translation status..."
	@uv run python scripts/check-translations.py || (echo "" && echo "   [!] Translation issues found (see above)" && echo "   Run with --verbose for details: uv run python scripts/check-translations.py --verbose")
	@echo ""
	@echo "========================================================================"
	@echo "[OK] Lint complete!"
	@echo "========================================================================"
	@echo ""
	@echo "Tips:"
	@echo "   - Check specific file: uv run python scripts/check-translations.py --file src/index.md"
	@echo "   - Show git diff: uv run python scripts/check-translations.py --show-diff"
	@echo "   - AI semantic audit: uv run python scripts/audit-translations-ai.py"

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
	uv run python scripts/audit-translations-ai.py
	echo ""
	echo "========================================================================"
	echo "[OK] Audit complete!"
	echo "========================================================================"
	echo ""
	echo "Tips:"
	echo "   - Use different model: uv run python scripts/audit-translations-ai.py --model opus"
	echo "   - Check specific file: uv run python scripts/audit-translations-ai.py --file src/index.md"
	echo "   - Enable verbose mode: uv run python scripts/audit-translations-ai.py --verbose"
