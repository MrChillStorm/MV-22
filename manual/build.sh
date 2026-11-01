#!/bin/sh
# Rebuild the flight manual from the aircraft's data (manual/data/level_trims.json is cached: mkmanual.py --trims
# recomputes it, from fdm/) and copy the PDF to the repository root, beside the model's report.
set -e
cd "$(dirname "$0")"
python3 mkmanual.py
python3 figures.py
tectonic -X compile mv22b_manual.tex
cp mv22b_manual.pdf ../MV22B-flight-manual.pdf
