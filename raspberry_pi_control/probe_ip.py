# Read-only: print the V2 status frame of each IP given on the command line.
import json, sys, urllib.parse, urllib.request
op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
for ip in sys.argv[1:]:
    url = f"http://{ip}/js?json=" + urllib.parse.quote(json.dumps({"T": 164}))
    try:
        print(ip, op.open(url, timeout=3).read().decode()[:300])
    except Exception as e:
        print(ip, "ERR", e)
