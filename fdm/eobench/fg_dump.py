"""launch FlightGear over LOWI (one click), freeze, and run Nasal/eodump.nas: the terrain and land cover round LOWI"""
import sys, os, time, shutil
sys.argv = ['x']
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fg_lowi.py')).read().split('import random')[0])
LAT, LON, HDG, KCAS, ALT = 47.26023, 11.34397, 139, 200, 6000
class FgDump(Fg):
    def start(s):
        args = [C.FG, '--fg-root=' + C.FG_ROOT, '--fg-aircraft=' + C.ROOT, '--aircraft=mv22b', '--lat=%.6f' % LAT, '--lon=%.6f' % LON,
                '--heading=%d' % HDG, '--altitude=%d' % ALT, '--vc=%d' % KCAS, '--wind=0@0', '--timeofday=noon', '--telnet=%d' % PORT,
                '--compositor=Compositor/HDR/hdr', '--ignore-autosave', '--geometry=%dx%d' % (C.W, C.H)]
        s.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('FlightGear launched (pid %d): waiting for the click' % s.proc.pid, flush=True)
        for _ in range(300):
            try:
                s.sock = socket.create_connection(('localhost', PORT), timeout=5); return
            except OSError:
                time.sleep(2)
        sys.exit('FlightGear did not open its telnet port')
fg = FgDump(launch=True)
fg.wait_ready()
fg.put('/sim/freeze/master', 1)
print('ready and frozen; letting the scenery load', flush=True)
time.sleep(25)
fg.put('/sim/eodump/load', 1)
t0 = time.time()
while fg.get('/sim/eodump/done') != 'true' and time.time() - t0 < 600:
    time.sleep(3)
    print('row', fg.get('/sim/eodump/row'), flush=True)
src = os.path.join(os.path.expanduser('~/Library/Application Support/FlightGear/Export'), 'lowi_dem.txt')
shutil.copy(src, os.path.join(S, 'lowi_dem.txt'))
print('copied', os.path.getsize(src), 'bytes; FlightGear left running (frozen)', flush=True)
