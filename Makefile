.PHONY: default setup build serve clean extract-messages

default:
	@echo 'Usage:'
	@echo '    make setup'
	@echo '        Install mdbook and mdbook-i18n-helpers'
	@echo
	@echo '    make build'
	@echo '        Build all language versions'
	@echo
	@echo '    make serve'
	@echo '        Serve all language versions at http://localhost:3000'
	@echo
	@echo '    make clean'
	@echo '        Remove all build outputs'
	@echo
	@echo '    make extract-messages'
	@echo '        Extract translatable messages to po/zh-TW.po'

setup:
	@command -v mdbook >/dev/null 2>&1 || cargo install mdbook --locked
	@command -v mdbook-i18n-helpers >/dev/null 2>&1 || cargo install mdbook-i18n-helpers --locked
	@mkdir -p po

build: extract-messages
	@mdbook build
	@if [ -f "po/zh-TW.po" ] && [ -f "book-zh-TW.toml" ]; then \
		cp book.toml book.toml.backup; \
		cp book-zh-TW.toml book.toml; \
		mdbook build -d book-zh-TW; \
		cp book.toml.backup book.toml; \
		rm book.toml.backup; \
		mkdir -p book/zh-TW; \
		cp -r book-zh-TW/* book/zh-TW/; \
		rm -rf book-zh-TW; \
	fi

serve: build
	@echo '<!DOCTYPE html><html><head><meta charset="UTF-8"><title>AutoSDV Book</title><style>body{font-family:sans-serif;display:flex;justify-content:center;align-items:center;height:100vh;margin:0;background:#f0f0f0}.container{text-align:center;padding:40px;background:white;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}h1{color:#333;margin-bottom:30px}a{display:inline-block;margin:10px;padding:15px 30px;background:#4CAF50;color:white;text-decoration:none;border-radius:5px;font-size:18px}a:hover{background:#45a049}</style></head><body><div class="container"><h1>AutoSDV Book</h1><a href="./index.html">🇬🇧 English</a><a href="./zh-TW/index.html">🇹🇼 繁體中文</a></div></body></html>' > book/index-select.html
	@echo ""
	@echo "══════════════════════════════════════════════════════"
	@echo "  AutoSDV Book Server Running"
	@echo "══════════════════════════════════════════════════════"
	@echo ""
	@echo "  Language Selection:  http://localhost:3000/index-select.html"
	@echo "  English Direct:      http://localhost:3000/"
	@echo "  Chinese Direct:      http://localhost:3000/zh-TW/"
	@echo ""
	@echo "  Press Ctrl+C to stop the server"
	@echo "══════════════════════════════════════════════════════"
	@cd book && python3 -m http.server 3000

clean:
	@rm -rf book book-zh-TW book-combined po/*.pot

extract-messages:
	@MDBOOK_OUTPUT='{"xgettext": {"pot-file": "po/messages.pot"}}' mdbook build -d po
	@if [ -f "po/messages.pot" ]; then \
		if [ -f "po/zh-TW.po" ]; then \
			msgmerge --update --no-fuzzy-matching po/zh-TW.po po/messages.pot; \
		else \
			msginit -i po/messages.pot -l zh_TW.UTF-8 -o po/zh-TW.po --no-translator; \
		fi \
	fi
	@rm -rf po/book po/messages.pot
