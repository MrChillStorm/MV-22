# sub.sh TAG SETS : the failing starts
export PATH=/opt/homebrew/bin:$PATH FIELD=1 BENCH_WIND="7 292" SETS="${2:-{\}}"
python3 nbatch.py 40 1 4 6 7 16 18 20 23 31 33 35 37 38 > s_$1.txt 2>&1
echo "$1 $2: $(grep -cE '^\s*[0-9]+\s+[0-9.]+ NM.*INTACT' s_$1.txt) intact of $(grep -cE '^\s*[0-9]+\s+[0-9.]+ NM' s_$1.txt)"
grep -E '^\s*[0-9]+\s+[0-9.]+ NM' s_$1.txt | sed -E 's/^ *([0-9]+) .*lb: (INTACT|not intact) +([0-9.]+) ft.s ([0-9]+) KCAS tips +([-+0-9.]+), on ([A-Za-z_]+) +slope +([-+0-9.]+) %, stop +([-+0-9]+).*down ([-+0-9]+).*/\1:\3\/\4\/\5\/\8\/\9/' | tr '\n' ' '; echo
