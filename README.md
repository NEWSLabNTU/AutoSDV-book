<p align="center">
  <img src="logo/logo_brand_gray.png" width=""/>
  <br>
  <a href="https://newslabntu.github.io/autosdv-book/">
    <strong>Read the Book »</strong>
  </a>
</p>

This repository contains the Markdown source files for the AutoSDV documentation.

## Quick Start

```bash
# Install dependencies
just setup

# Build documentation
just build

# Serve locally (with live reload)
just serve
```

Documentation will be available at http://localhost:3000

## Project Structure

```
book/
├── src/                        # Markdown source files
│   ├── index.md               # Homepage
│   ├── getting-started/       # Getting started guides
│   ├── guides/                # Tutorial guides
│   └── reference/             # Technical reference
├── scripts/                   # Translation checking tools
├── mkdocs.yml                # MkDocs configuration
└── justfile                  # Build commands
```

## Contributing

### Writing Documentation

1. Edit markdown files in `src/`
2. Build and preview: `just serve`
3. Validate: `just lint`
4. Submit PR

### Translating to Chinese

2. Create `.zh-TW.md` file alongside English version
3. Translate content (see [CLAUDE.md](CLAUDE.md#translation-workflow))
4. Validate: `just lint`
5. Submit PR

## Commands

```bash
just              # List all commands
just setup        # Install dependencies
just build        # Build documentation
just serve        # Serve with live reload
just lint         # Validate format and translations
just clean        # Remove build outputs
```

## Technology

- **Framework**: MkDocs with Material theme
- **i18n**: mkdocs-static-i18n (English + Traditional Chinese)
- **Search**: Built-in with jieba for Chinese
- **Build**: Just command runner
