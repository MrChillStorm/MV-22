import re,sys,statistics as st
def rd(f):
    d={}
    for l in open(f):
        m=re.match(r'\s*(\d+)\s+[\d.]+ NM.*lb: (INTACT|not intact)\s+([\d.]+) ft/s\s+(\d+) KCAS.*on (\S+)\s+slope\s+(\S+) %, stop\s+([-+]\d+)',l)
        if m: d[int(m.group(1))]=(m.group(2)=='INTACT',float(m.group(3)),int(m.group(4)),m.group(5),int(m.group(7)))
    return d
for f in sys.argv[1:]:
    d=rd(f); s=[v[4] for v in d.values()]
    print(f,'n',len(s),'intact',sum(v[0] for v in d.values()),'stop past end: over 0:',sum(x>0 for x in s),'over 1000:',sum(x>1000 for x in s),'over 3000:',sum(x>3000 for x in s),'worst',max(s),'median',st.median(s),'sink>10:',sum(v[1]>10 for v in d.values()),'median sink',st.median(v[1] for v in d.values()))
    print('   not intact',[(k,d[k][1],d[k][2],d[k][3]) for k in sorted(d) if not d[k][0]])
