import json, sys, os
dec = json.JSONDecoder()
src = sys.argv[1]; out = sys.argv[2]
for name, fn in [('run1', 'Experiment0_Report_EN.html'), ('last40', 'Last40_Report_EN.html')]:
    t = open(os.path.join(src, fn), encoding='utf-8').read()
    r = dec.raw_decode(t[t.index('const RECS=') + 11:])[0][0]
    json.dump({'opt': r['opt'], 'mm': r['mm'], 'stops': r['stops']}, open(os.path.join(out, name + '.json'), 'w'))
    print(name, len(r['opt']['t']), len(r['mm']['t']))
