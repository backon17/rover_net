# Read-only: HTTP round-trip latency to each car (T:164 status), N samples.
import json, sys, time, urllib.parse, urllib.request
op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
n = int(sys.argv[1])
for ip in sys.argv[2:]:
    url = f"http://{ip}/js?json=" + urllib.parse.quote(json.dumps({"T": 164}))
    rtts, fails = [], 0
    for _ in range(n):
        t = time.perf_counter()
        try:
            op.open(url, timeout=1.0).read()
            rtts.append((time.perf_counter() - t) * 1000)
        except Exception:
            fails += 1
        time.sleep(0.05)
    rtts.sort()
    q = lambda p: rtts[min(len(rtts) - 1, int(p * len(rtts)))] if rtts else -1
    gaps = sum(1 for r in rtts if r > 400)
    print(f"{ip}: n={len(rtts)} fail={fails} p50={q(.5):.0f}ms p90={q(.9):.0f}ms "
          f"p99={q(.99):.0f}ms max={rtts[-1] if rtts else -1:.0f}ms  >400ms: {gaps}")
