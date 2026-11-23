import sys, json, time, urllib.parse, urllib.request, statistics as st
IP = sys.argv[1]
def u(o):
    r = urllib.request.urlopen(f"http://{IP}/js?json=" + urllib.parse.quote(json.dumps(o)), timeout=1).read()
    return json.loads(r) if r.strip() else None
g = {"gx": [], "gy": [], "gz": [], "ax": [], "ay": [], "az": []}
t0 = time.monotonic()
while time.monotonic() - t0 < 6:
    d = u({"T": 126})
    if d:
        for k in g: g[k].append(d[k])
    time.sleep(0.02)
print(len(g["gz"]), "samples")
for k, v in g.items():
    s = sorted(v)
    print(f"{k}: mean {st.mean(v):8.2f} std {st.pstdev(v):6.2f} min {s[0]:8.2f} p50 {s[len(s)//2]:8.2f} max {s[-1]:8.2f}")
print("gz first 40:", [round(x, 1) for x in g["gz"][:40]])
