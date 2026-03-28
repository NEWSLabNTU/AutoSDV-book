default:
	@just --list

setup:
	#!/usr/bin/env bash
	set -e
	echo "Installing MkDocs and dependencies..."
	# Auto-install uv if not found
	if ! command -v uv &>/dev/null; then
		echo "→ uv not found. Installing..."
		curl -LsSf https://astral.sh/uv/install.sh | sh
		export PATH="$HOME/.local/bin:$PATH"
		echo "✓ uv $(uv --version) installed"
	fi
	uv sync
	echo ""
	echo "[OK] Setup complete! (mkdocs, mike, and all plugins installed)"
	echo "  Run 'just build' to build the documentation"
	echo "  Run 'just serve' to serve the documentation"

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

# Versioned deployment using mike
#
# Version scheme: X.Y-N (X.Y = AutoSDV version, N = book revision)
# Branches: main (dev), 0.1 (AutoSDV 0.1.x docs), 0.2 (AutoSDV 0.2.x docs)
# Tags: 0.1-1, 0.1-2, 0.2-1, etc.
#
# Usage:
#   just deploy-release 0.1 0.1-1    Deploy version 0.1 as latest
#   just deploy-dev                   Deploy dev from main branch
#   just deploy-list                  List all deployed versions

# Deploy a release version (run from a version branch)
deploy-release version tag:
	@echo "Deploying {{version}} ({{tag}}) as latest..."
	@uv run mike deploy --push --update-aliases "{{version}}" latest --title "{{version}} ({{tag}})"
	@uv run mike set-default --push latest
	@echo "[OK] Deployed {{version}} as latest"

# Deploy development version (run from main branch)
deploy-dev:
	@echo "Deploying dev version..."
	@uv run mike deploy --push --update-aliases dev --title "dev (unreleased)"
	@echo "[OK] Deployed dev version"

# Set which version / is redirected to
deploy-set-default alias="latest":
	@echo "Setting default to {{alias}}..."
	@uv run mike set-default --push "{{alias}}"
	@echo "[OK] Default set to {{alias}}"

# List all deployed versions
deploy-list:
	@uv run mike list

# Serve all deployed versions locally (reads from gh-pages branch)
serve-versions:
	@echo "Serving all deployed versions at http://localhost:8000"
	@uv run mike serve

# Check dependencies, validate docs, and check translations
check:
	#!/usr/bin/env bash
	set -e
	echo "========================================================================"
	echo "Documentation Check"
	echo "========================================================================"
	echo ""
	echo "[1/3] Checking dependencies..."
	uv run python -c "import mkdocs; print('      [OK] mkdocs')" 2>/dev/null || { echo "      [X] mkdocs not found (run 'just setup')"; exit 1; }
	uv run python -c "import material; print('      [OK] mkdocs-material')" 2>/dev/null || { echo "      [X] mkdocs-material not found (run 'just setup')"; exit 1; }
	uv run python -c "import mkdocs_awesome_pages_plugin; print('      [OK] mkdocs-awesome-pages-plugin')" 2>/dev/null || { echo "      [X] mkdocs-awesome-pages-plugin not found (run 'just setup')"; exit 1; }
	uv run python -c "import mkdocs_static_i18n; print('      [OK] mkdocs-static-i18n')" 2>/dev/null || { echo "      [X] mkdocs-static-i18n not found (run 'just setup')"; exit 1; }
	echo ""
	echo "[2/3] Validating MkDocs configuration..."
	uv run mkdocs build --strict --quiet && echo "      [OK] MkDocs build successful (strict mode)" || { echo "      [X] MkDocs build failed"; exit 1; }
	echo ""
	echo "[3/3] Checking translation status..."
	uv run python scripts/check-translations.py || { echo ""; echo "      [!] Translation issues found (see above)"; echo "      Run with --verbose for details: uv run python scripts/check-translations.py --verbose"; }
	echo ""
	echo "========================================================================"
	echo "[OK] Check complete!"
	echo "========================================================================"
	echo ""
	echo "Tips:"
	echo "   - Check specific file: uv run python scripts/check-translations.py --file src/index.md"
	echo "   - Show git diff: uv run python scripts/check-translations.py --show-diff"
	echo "   - AI semantic audit: just audit-translations"

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
