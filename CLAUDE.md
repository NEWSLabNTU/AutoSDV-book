# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands
- Build: `make build` - Compiles the HTML document using mdbook
- Serve: `make serve` - Starts a web server at http://127.0.0.1:3000 with the compiled book
- Clean: `make clean` - Removes the compiled HTML document

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