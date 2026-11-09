import sys, math
sys.path.insert(0, '.')
import deadstick as D
nac, V, gear, fl = (float(x) for x in sys.argv[1:5])
try:
    r = D.glide((nac, V, 38722.0, gear, fl))[2]
except Exception as e:
    print('gear %d flap %2.0f nac %2.0f %3.0f KCAS: failed %s' % (gear, fl, nac, V, str(e)[:60]), flush=True); sys.exit()
if r is None:
    print('gear %d flap %2.0f nac %2.0f %3.0f KCAS: no trim' % (gear, fl, nac, V), flush=True); sys.exit()
g, th, e = r
ld = 1 / math.tan(-math.radians(g)) if g < 0 else 99
print('gear %d flap %2.0f nac %2.0f %3.0f KCAS: L/D %5.2f gamma %5.1f theta %5.1f res %.3f sink %3.0f fps' % (gear, fl, nac, V, ld, g, th, e, V * 1.6878 * math.sin(-math.radians(g))), flush=True)
