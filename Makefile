.PHONY: default build serve clean

default:
	@echo 'Usage:'
	@echo '    make build'
	@echo '        Compile the HTML document'
	@echo
	@echo '    make serve'
	@echo '        Start a web server running at http://127.0.0.1:3000'
	@echo
	@echo '    make clean'
	@echo '        Remove the compiled HTML document'

build:
	mdbook build

serve:
	mdbook serve --open

clean:
	rm -rf book
