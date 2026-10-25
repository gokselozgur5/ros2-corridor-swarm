"""Real model, fake ROS: the planner's Claude side against claude-opus-5-5 with stubbed tools. Runs anywhere with
the anthropic package, no ROS needed: python3 tools/planner_smoke.py "send v1 to patrol" [effort].
Prints the trace, the reply, tokens and cost. Run with ANTHROPIC_API_KEY in the environment."""
import json
import sys
import time

import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ws', 'src', 'corridor_planner'))
import anthropic
from corridor_planner.planner import Planner
from corridor_planner.tools import SCHEMAS

PRICE = {'claude-opus-5-5': (4.0, 20.0)}
fleet = {'v1': {'mode': 'idle', 'follower': 'idle', 'lat': 52.70018, 'lon': -2.99971, 'inside_corridor': True, 'edge_distance_m': 20.0, 'x': 20.0, 'y': 20.0, 'speed_mps': 0.0},
         'v2': {'mode': 'patrol', 'follower': 'following', 'lat': 52.70090, 'lon': -2.99850, 'inside_corridor': True, 'edge_distance_m': 60.0, 'x': 101.0, 'y': 100.0, 'speed_mps': 3.0}}
calls = []
tools = {
    'list_vehicles': lambda: {'corridor_id': 'demo', 'vehicles': [dict(vehicle=k, **v) for k, v in fleet.items()]},
    'get_vehicle': lambda vehicle: dict(vehicle=vehicle, **fleet[vehicle]),
    'patrol': lambda vehicle, laps, clockwise: calls.append(('patrol', vehicle, laps, clockwise)) or {'accepted': True, 'vehicle': vehicle, 'laps': laps, 'clockwise': clockwise},
    'goto': lambda vehicle, lat, lon: calls.append(('goto', vehicle, lat, lon)) or {'accepted': True},
    'stop': lambda vehicle: calls.append(('stop', vehicle)) or {'stopped': True, 'vehicle': vehicle, 'cancelled_patrol': fleet[vehicle]['mode'] == 'patrol'},
    'geofence_events': lambda limit: [{'vehicle': 'v2', 'type': 'warn', 'distance_m': 14.2, 'stamp': 1792850000}],
    'describe_scene': lambda vehicle: {'vehicle': vehicle, 'width': 320, 'height': 240, 'frame_age_s': 0.1, 'png_bytes': 0,
                                       '_content': [{'type': 'text', 'text': f'{vehicle} camera: (no frame on the Mac smoke test)'}]},
}
p = Planner(anthropic.Anthropic(), tools=tools, schemas=SCHEMAS, effort=sys.argv[2] if len(sys.argv) > 2 else 'medium')
t0 = time.time()
out = p.run(sys.argv[1] if len(sys.argv) > 1 else 'send v1 to patrol the corridor, one lap')
dt = time.time() - t0
for ev in p.trace:
    e = dict(ev)
    if e['event'] == 'tool':
        e['result'] = json.dumps(e['result'])[:120]
    print(json.dumps(e)[:300])
cin, cout = PRICE['claude-opus-5-5']
print(f'\nreply: {out.text}\nrefused={out.refused} category={out.category!r} error={out.error!r}')
print(f'turns={out.turns} tool_calls={out.tool_calls} in={out.input_tokens} out={out.output_tokens} '
      f'cost=${(out.input_tokens*cin + out.output_tokens*cout)/1e6:.4f} time={dt:.1f}s')
print('tool side effects:', calls)
