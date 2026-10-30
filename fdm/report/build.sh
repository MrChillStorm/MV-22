#!/bin/sh
# Rebuild the report from the cached results (report/results/*.json; rerun report/mkresults.py to refresh them)
# and copy the PDF to the repository root.
set -e
cd "$(dirname "$0")"
python3 mktables.py
python3 figures.py
tectonic -X compile v22_report.tex
cp v22_report.pdf ../../MV22B-JSBSim-model-report.pdf
