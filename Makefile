# === Compiler and Tool Variables ===
DOXYGEN       = doxygen
PDFLATEX      = pdflatex

# === Output Directories ===
OUTPUT_DIR    = docs
LATEX_DIR     = $(OUTPUT_DIR)/latex

# === Image Assets ===
IMAGE_FILES   = drag_model_chart_combined.png

# === Default Target ===
all: doc-pdf

# === Run Doxygen to Generate Docs ===
doc:
	$(DOXYGEN) Doxyfile

# === Clean Documentation Outputs ===
doc-clean:
	rm -rf $(OUTPUT_DIR)/html $(LATEX_DIR) $(LATEX_DIR)/refman.pdf

# === Generate PDF Documentation with Image Support ===
doc-pdf: doc
	@if [ ! -d "$(LATEX_DIR)" ]; then \
		echo "Error: '$(LATEX_DIR)' not found. Did Doxygen fail?"; \
		exit 1; \
	fi
	cp $(IMAGE_FILES) $(LATEX_DIR)/
	cd $(LATEX_DIR) && $(PDFLATEX) refman.tex && $(PDFLATEX) refman.tex

# === View the PDF Output ===
view:
	xdg-open $(LATEX_DIR)/refman.pdf

# === Clean Everything (including LaTeX build junk) ===
clean: doc-clean
	rm -f *.aux *.log *.toc *.out *.gz core
