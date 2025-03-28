# === Compiler and Tool Variables ===
CC            = gcc
CFLAGS        = -Wall -Wextra -std=c23 -O3
DOXYGEN       = doxygen
PDFLATEX      = pdflatex

# === Directories and Files ===
SRC_DIR       = src
INCLUDE_DIR   = include
OUTPUT_DIR    = docs
LATEX_DIR     = $(OUTPUT_DIR)/latex
IMAGE_FILES   = drag_model_chart_combined.png

# === Source Files and Executable ===
SRCS          = $(wildcard $(SRC_DIR)/*.c)
OBJS          = $(SRCS:.c=.o)
TARGET        = bbtc

# === Default Target ===
all: $(TARGET)

# === Compile Executable ===
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -o $@ $(OBJS) -lm

# === Compile source files into object files ===
%.o: %.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c $< -o $@

# === Documentation Generation ===
doc:
	$(DOXYGEN) Doxyfile

# === PDF Documentation with Image Support ===
doc-pdf: doc
	@if [ ! -d "$(LATEX_DIR)" ]; then \
		echo "Error: '$(LATEX_DIR)' not found. Did Doxygen fail?"; \
		exit 1; \
	fi
	cp $(IMAGE_FILES) $(LATEX_DIR)/
	cd $(LATEX_DIR) && $(PDFLATEX) refman.tex && $(PDFLATEX) refman.tex

# === View PDF Documentation ===
view:
	xdg-open $(LATEX_DIR)/refman.pdf

# === Clean Targets ===
clean:
	rm -f $(OBJS) $(TARGET)
	rm -f *.aux *.log *.toc *.out *.gz core
	rm -rf $(OUTPUT_DIR)/html $(LATEX_DIR)

.PHONY: all clean doc doc-pdf view
