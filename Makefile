.PHONY: default setup build serve clean help

default: help

help:
	@echo 'AutoSDV Book - MkDocs Documentation'
	@echo ''
	@echo 'Usage:'
	@echo '    make setup'
	@echo '        Install MkDocs and dependencies'
	@echo ''
	@echo '    make build'
	@echo '        Build all language versions (English + 繁體中文)'
	@echo ''
	@echo '    make serve'
	@echo '        Serve documentation at http://localhost:8000'
	@echo ''
	@echo '    make clean'
	@echo '        Remove all build outputs'
	@echo ''
	@echo 'Advanced:'
	@echo '    make build-en       Build English version only'
	@echo '    make build-zh       Build Chinese version only'
	@echo '    make serve-en       Serve English version only'
	@echo '    make serve-zh       Serve Chinese version only'

setup:
	@echo "Installing MkDocs and dependencies..."
	@command -v pip3 >/dev/null 2>&1 || (echo "Error: pip3 not found. Please install Python 3 and pip3." && exit 1)
	@pip3 install -r requirements.txt
	@echo ""
	@echo "✓ Setup complete!"
	@echo "  Run 'make build' to build the documentation"
	@echo "  Run 'make serve' to serve the documentation"

build:
	@echo "Building all language versions..."
	@mkdocs build
	@echo ""
	@echo "✓ Build complete!"
	@echo "  Output: site/"
	@echo "  English:  site/en/"
	@echo "  Chinese:  site/zh-TW/"
	@echo ""
	@echo "  Open site/index.html to view the documentation"

build-en:
	@echo "Building English version only..."
	@mkdocs build -f mkdocs.yml
	@echo ""
	@echo "✓ Build complete!"
	@echo "  Output: site/"

build-zh:
	@echo "Building Chinese version only..."
	@mkdocs build -f mkdocs.yml
	@echo ""
	@echo "✓ Build complete!"
	@echo "  Output: site/"

serve:
	@echo ""
	@echo "══════════════════════════════════════════════════════"
	@echo "  AutoSDV Book Server Running"
	@echo "══════════════════════════════════════════════════════"
	@echo ""
	@echo "  Documentation:      http://localhost:8000"
	@echo "  English:            http://localhost:8000/en/"
	@echo "  繁體中文:            http://localhost:8000/zh-TW/"
	@echo ""
	@echo "  Press Ctrl+C to stop the server"
	@echo "══════════════════════════════════════════════════════"
	@echo ""
	@mkdocs serve -a 0.0.0.0:3000

clean:
	@echo "Cleaning build outputs..."
	@rm -rf site
	@echo "✓ Clean complete!"

# Development helpers
watch:
	@echo "Watching for changes and rebuilding..."
	@mkdocs serve --watch src

deploy:
	@echo "Deploying to GitHub Pages..."
	@mkdocs gh-deploy --force
	@echo "✓ Deployment complete!"

# Check if dependencies are installed
check:
	@echo "Checking MkDocs installation..."
	@command -v mkdocs >/dev/null 2>&1 && echo "✓ mkdocs found" || echo "✗ mkdocs not found (run 'make setup')"
	@python3 -c "import mkdocs_material" 2>/dev/null && echo "✓ mkdocs-material found" || echo "✗ mkdocs-material not found (run 'make setup')"
	@python3 -c "import mkdocs_awesome_pages_plugin" 2>/dev/null && echo "✓ mkdocs-awesome-pages-plugin found" || echo "✗ mkdocs-awesome-pages-plugin not found (run 'make setup')"
	@python3 -c "import mkdocs_static_i18n" 2>/dev/null && echo "✓ mkdocs-static-i18n found" || echo "✗ mkdocs-static-i18n not found (run 'make setup')"
