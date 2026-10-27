#!/usr/bin/env bash
# Issue #16, server side only: cot_bridge streams PLI for v1..v3 to the local FreeTAKServer, and a second CoT client (an ATAK
# stand-in) receives them back from the server. A TCP handshake proves nothing: the bridge reconnects silently when the
# server drops it, so the verdict needs the relayed events and exactly one connection.
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
source ./steps/lib.sh
LOG=~/deletdis/log/13_cot_proof.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) cot proof"
cleanup() { kill_ours; sleep 1; }
cleanup
ss -ltn | grep -q ":8087" || { echo "no TAK server on 8087"; echo "PROOF 16 FAIL"; exit 1; }
ros2 launch corridor_nav kinematic.launch.py n:=3 > /tmp/kin16.log 2>&1 &
sleep 6
ros2 run corridor_cot cot_bridge --ros-args -p "vehicles:=[v1,v2,v3]" -p host:=127.0.0.1 -p period_s:=1.0 > /tmp/cot16.log 2>&1 &
sleep 8
grep -m1 "connected to TAK server" /tmp/cot16.log || echo "bridge did not connect"
before=$(sudo -n docker logs corridor-fts 2>&1 | wc -l)
# Second client, like ATAK: register with its own PLI, then collect what the server relays for 15 s.
python3 - <<'PY' 2>&1 | tee /tmp/relay16.txt
import collections, datetime, re, socket, time
now = datetime.datetime.now(datetime.timezone.utc)
f = lambda t: t.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
hello = (f'<event version="2.0" uid="atak-listener-1" type="a-f-G-U-C" how="h-e" time="{f(now)}" start="{f(now)}" '
         f'stale="{f(now + datetime.timedelta(minutes=5))}"><point lat="52.7005" lon="-2.9990" hae="0" ce="9999999" le="9999999"/>'
         '<detail><contact callsign="LISTENER"/><__group name="Cyan" role="Team Member"/></detail></event>')
s = socket.create_connection(('127.0.0.1', 8087), timeout=5)
s.sendall(hello.encode())
seen, buf, closed = collections.Counter(), b'', False
deadline = time.time() + 15
while time.time() < deadline:
    s.settimeout(max(0.1, deadline - time.time()))
    try:
        chunk = s.recv(65536)
    except socket.timeout:
        continue
    except OSError:
        closed = True
        break
    if not chunk:
        closed = True
        break
    buf += chunk
    while b'</event>' in buf:
        xml, buf = buf.split(b'</event>', 1)
        m = re.search(rb'<event\b[^>]*\buid="([^"]*)"', xml)
        if m:
            seen[m.group(1).decode()] += 1
s.close()
print('events relayed by the server to the second client in 15 s: ' + (' '.join(f'{u}={n}' for u, n in sorted(seen.items())) or 'none')
      + (' (server closed the listener socket)' if closed else ''))
# the bridge sends each vehicle once a second: a cached registration event alone is one, a live stream is many
missing = [u for u in ('corridor-v1', 'corridor-v2', 'corridor-v3') if seen[u] < 3]
print('RELAY PASS' if not missing else 'RELAY FAIL fewer than 3 events for ' + ' '.join(missing))
PY
sudo -n docker logs corridor-fts 2>&1 | tail -n +$before | grep -i -m3 "corridor-v\|atak-listener\|error" | cut -c1-160
connects=$(grep -c "connected to TAK server" /tmp/cot16.log 2>/dev/null || true); connects=${connects:-0}
echo "bridge connections: $connects (more than 1 means the server dropped the session)"
grep -q "^RELAY PASS" /tmp/relay16.txt && [ "$connects" -eq 1 ] && ! grep -q "TAK send failed" /tmp/cot16.log \
  && echo "PROOF 16 PASS (server side: PLI for v1..v3 relayed to a second client; ATAK display needs the emulator)" \
  || { echo "PROOF 16 FAIL"; tail -5 /tmp/cot16.log; }
cleanup
echo "=== $(date -Is) done"
