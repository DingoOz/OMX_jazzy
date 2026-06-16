#!/usr/bin/env bash
# Build the dynamixel_demo report PDF.
#
# Usage:  ./build_pdf.sh
# Output: build/dynamixel_demo.pdf
set -euo pipefail

cd "$(dirname "$0")"

if command -v latexmk >/dev/null 2>&1; then
    latexmk -pdf -outdir=build dynamixel_demo.tex
else
    # Fallback: two pdflatex passes so the table of contents resolves.
    mkdir -p build
    pdflatex -output-directory=build dynamixel_demo.tex
    pdflatex -output-directory=build dynamixel_demo.tex
fi

echo "PDF written to $(pwd)/build/dynamixel_demo.pdf"
