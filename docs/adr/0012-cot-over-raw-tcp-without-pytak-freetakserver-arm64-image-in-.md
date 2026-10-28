# ADR-0012: CoT over raw TCP without pytak; FreeTAKServer arm64 image in a named Docker container

Status: Accepted (code 2026-10-24, commits 90ea32c, 244424b; container on Thor 2026-10-24 per issue #15; single-event registration fix 2026-10-27, 1c99eb8, merged in 60633c8)

## Context
Phase 4 (PLAN.md 63-66) needs the fleet in ATAK: PLI out, markers back as `/mission/request`. ARCHITECTURE.md line 78 planned `cot_bridge (pytak)`. `freetakteam/freetakserver` on Docker Hub had no tags left when the container was set up on 2026-10-24; Thor is aarch64 (ARCHITECTURE.md line 85, "Gazebo, not Isaac Sim"), so the amd64-only `rlex/freetakserver:1.0.3.1` was out (issue #15 comment, 2026-10-24). On the first Thor the user was not in the docker group and passwordless sudo worked (issue #15, 2026-10-24); the work Thor (.55) carries other people's docker stacks and its user is in the docker group with no passwordless sudo (commit eca1a62; issue #15 comment of 2026-10-28 describing the 2026-10-27 setup). The 8087 stream is plain TCP, no TLS, unauthenticated (cot_bridge.py docstring lines 3-4; cot.py `marker_or_reason` docstring lines 67-69).

## Decision
- `corridor_cot` builds and parses CoT XML with `xml.etree` only; no pytak, no asyncio (cot.py line 1 docstring, imports lines 2-5; package.xml depends only on rclpy, sensor_msgs, nav_msgs, std_msgs). Commit 90ea32c "no pytak".
- Node `cot_bridge` (cot_bridge.py): subscribes `/{v}/fix` (NavSatFix, ignored unless `status >= STATUS_FIX`, lines 57-60) and `/{v}/odom` (course via `course_from_quaternion`, speed = |linear.x|, lines 38-40); one `socket.create_connection((host, port), timeout=5.0)`, then `settimeout(None)` (lines 47-52); `sendall` of PLI events on a `period_s` timer (line 44); a reader thread splits the stream on `</event>` (lines 98-104); markers (`b-m-p*`, `a-u-G*`, `a-h-G*`, cot.py line 77; uid not `corridor-*`, cot_bridge.py line 112) become `std_msgs/String` on `/mission/request`. An `OSError` on send (lines 76-78) or an empty/failed `recv` (lines 88-96) sets `sock = None`; the next tick reconnects. Params: `vehicles` [v1], `host` ($FTS_HOST, default 127.0.0.1), `port` 8087, `period_s` 2.0, `fix_timeout_s` 5.0, `marker_move_m` 5.0, `team` Cyan (lines 27-33).
- A (re)connection's first write is exactly one event; the full batch follows from the next tick (cot_bridge.py lines 69-75, commit 1c99eb8). FTS 1.9.9.2 reads a connection's first message up to `</event>` as the client registration and drops a client whose first chunk holds more (1c99eb8 body; cot_bridge.py comment lines 71-73).
- `MarkerGate` (cot.py 111-135): a uid passes the first time and again only after moving `move_m`; at most 256 uids remembered (`capacity=256`); a marker without a uid always passes (commits 883fde6, c81a5b4).
- Server: `vigsecdrone/freetakserver:1.9.9.2` (amd64 and arm64), container named `corridor-fts`, never touching other containers (PLAN.md 63). First Thor: `-p 8087 -p 8089 -p 19023 --restart unless-stopped`; work Thor .55: bound to `127.0.0.1:8087` only, no restart policy (issue #15 comments 2026-10-24 and 2026-10-28). steps/13 reads `docker logs corridor-fts` with plain docker when `docker ps` works, else `sudo -n docker logs` (steps/13_cot_proof.sh line 18, commit eca1a62).

## Alternatives considered
- **pytak** (ARCHITECTURE.md line 78, stale): not used since 90ea32c; the XML builder/parser is unit-testable and dependency-free without it (cot.py line 1). No evaluation of pytak itself is recorded in the repo.
- **`freetakteam/freetakserver` official image**: no tags on Docker Hub at setup time (issue #15, 2026-10-24). **`rlex/freetakserver:1.0.3.1`**: amd64 only, Thor is arm64 (issue #15).
- **FTS from pip in a venv**: listed in issue #15 as an option; nothing in the repo evaluates it.
- **TLS on 8089**: port mapped on the first Thor only; the bridge uses plain 8087 (cot_bridge.py line 3). Nothing in the repo evaluates TLS.

## Consequences
- Good: zero extra Python deps; `test_cot.py` (11 tests) and `test_cot_bridge_node.py` (7 node tests) run without a server; "cot 7 node tests" pass on thor .55 (60633c8 body).
- Good: steps/13 proves the relay end to end: a second TCP client sees `corridor-v1..v3` with at least 3 events each in 15 s, exactly one bridge connection, no `TAK send failed` (steps/13 lines 54-62; run of 2026-10-27 in issue #16: 15 events per vehicle, 1 connection). steps/14 proves a marker from a second client reaches `/mission/request` (RALLY1, lines 28-30); it sends the marker twice but only checks that a request appears, the one-request-per-placement behaviour is pinned by `test_same_marker_twice_is_one_request_and_a_move_is_another` (test_cot_bridge_node.py 200).
- Bad: hand-rolled framing on `</event>`; a server-side drop is silent (reader sets `sock = None`, no log line, cot_bridge.py 92-96), which is why steps/13 counts `connected to TAK server` lines instead of trusting the handshake (edcd6fb). Before 1c99eb8 this hid 22 reconnects in 30 s with no PLI relayed (1c99eb8 body; issue #16 correction comment).
- Bad: unauthenticated 8087; any client's marker becomes a mission request, mitigated only by `MarkerGate`, the stale check and the lat/lon range check (cot.py 77-92, 111-135).
- Bad: FTS 1.9.9.2 pinned to a third-party image; the single-event registration is a behaviour observed on that version (1c99eb8 body).
- Docs drift: ARCHITECTURE.md lines 16, 47, 78 and PLAN.md line 65 still say `/fleet/state` and `pytak`; the code reads `/vN/fix` + `/vN/odom` and has no pytak. `/fleet/state` has no publisher (ARCHITECTURE.md line 100, open question 3).
- Still open: ATAK emulator proof for #15/#16 (issue comments of 2026-10-28; README.md 32).

## Evidence
- Commits: 8c201fe, 90ea32c, b65988a, 244424b, 71e69b7 (2026-10-24); 980bd92, f13940c (steps 13/14, 2026-10-24); 9846f0b, b53ac85, 022fe28, 33f5207, 8e0a6c8, 883fde6, adecbc0, c81a5b4, edcd6fb, cf203a4, 967c6c7 (2026-10-26 review); 1c99eb8, eca1a62, 60633c8 (2026-10-27).
- Files: ws/src/corridor_cot/corridor_cot/cot.py, cot_bridge.py, package.xml, test/test_cot.py, test/test_cot_bridge_node.py; steps/13_cot_proof.sh, steps/14_marker_proof.sh; PLAN.md 63-66; ARCHITECTURE.md 10-16, 47, 78, 85, 100; README.md 32, 47; .env.example (FTS_HOST).
- Issues: #15 (image choice, container run lines, .55 setup; comments 2026-10-24 x2, 2026-10-28), #16 (bridge; proofs 2026-10-24, correction 2026-10-28).
