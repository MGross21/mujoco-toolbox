# Minimal Makefile for Sphinx documentation

# Allow overriding from command line
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SPHINXPROJ    = MujocoToolboxDocsSphinx
SOURCEDIR     = .
BUILDDIR      = _build

.PHONY: help clean html livehtml

help:
	@echo "Please use 'make <target>' where <target> is one of:"
	@echo "  clean      - Remove all built documentation"
	@echo "  html       - Build the HTML documentation"
	@echo "  livehtml   - Serve docs with live reloading (requires sphinx-autobuild)"

clean:
	rm -rf $(BUILDDIR)
	@echo "Cleaned build directory."

html:
	$(SPHINXBUILD) -b html $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS)
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/html."

livehtml:
	@command -v sphinx-autobuild >/dev/null 2>&1 || { \
		echo >&2 "sphinx-autobuild is not installed. Install it with:"; \
		echo >&2 "  pip install sphinx-autobuild"; \
		exit 1; \
	}
	sphinx-autobuild $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS)