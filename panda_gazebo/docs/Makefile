# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = panda_openai_sim
SOURCEDIR     = source
BUILDDIR      = build
GH_PAGES_SOURCES = docs panda_openai_sim panda_training

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Publish gh-pages command
.PHONY: gh-pages
gh-pages:
	cd .. && \
	git submodule update --recursive && \
	git checkout gh-pages && \
	git rm -rf . && git clean -fxd && \
	git checkout melodic-devel $(GH_PAGES_SOURCES) && \
	git reset HEAD && \
	cd docs && \
	make clean && \
	make html && \
	cd .. && \
	git add docs/build/html && \
	git clean -ffxd && \
	mv -fv docs/build/html/* ./ && \
	rm -rf $(GH_PAGES_SOURCES) && \
	git add -A && \
	git commit -m ":pencil: Generated gh-pages for `git log melodic-devel -1 --pretty=short --abbrev-commit`" && \
	git push origin gh-pages && \
	git checkout melodic-devel && \
	git submodule update --recursive

# Create make clean command
.PHONY: clean
clean:
	@rm -rf $(BUILDDIR)/*
	@echo "Build folder cleaned"

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
