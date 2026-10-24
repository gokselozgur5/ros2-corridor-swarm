"""Planner against a stub client. No network, no ROS."""
import json
from types import SimpleNamespace as NS

from corridor_planner.planner import Planner
from corridor_planner.tools import SCHEMAS


class StubClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []
        self.beta = NS(messages=NS(create=self.create))

    def create(self, **kwargs):
        self.requests.append(kwargs)
        return self.responses.pop(0)


def usage(i=10, o=5):
    return NS(input_tokens=i, output_tokens=o)


def test_refusal_is_an_outcome_not_an_exception():
    client = StubClient([NS(stop_reason='refusal', stop_details=NS(category='cyber'), content=[], usage=usage(), model='m')])
    p = Planner(client, tools={}, schemas=SCHEMAS)
    out = p.run('do something')
    assert out.refused and out.category == 'cyber'
    assert p.trace[-1]['event'] == 'refusal'
    assert p.messages == []  # the refused request is not kept in history


def test_tool_loop_executes_and_feeds_back():
    calls = []
    tools = {'patrol': lambda vehicle, laps, clockwise: calls.append((vehicle, laps, clockwise)) or {'accepted': True}}
    client = StubClient([
        NS(stop_reason='tool_use', usage=usage(), content=[
            NS(type='tool_use', id='t1', name='patrol', input={'vehicle': 'v2', 'laps': 1, 'clockwise': False})]),
        NS(stop_reason='end_turn', usage=usage(), content=[NS(type='text', text='v2 is patrolling.')]),
    ])
    out = Planner(client, tools=tools, schemas=SCHEMAS).run('send v2 on a lap')
    assert calls == [('v2', 1, False)]
    assert out.text == 'v2 is patrolling.' and out.tool_calls == 1 and out.turns == 2
    second = client.requests[1]['messages']
    assert second[-1]['role'] == 'user' and second[-1]['content'][0]['tool_use_id'] == 't1'
    assert second[-1]['content'][0]['is_error'] is False


def test_tool_exception_becomes_error_result():
    def boom(vehicle):
        raise RuntimeError('no such vehicle')
    client = StubClient([
        NS(stop_reason='tool_use', usage=usage(), content=[NS(type='tool_use', id='t1', name='stop', input={'vehicle': 'v9'})]),
        NS(stop_reason='end_turn', usage=usage(), content=[NS(type='text', text='v9 does not exist.')]),
    ])
    p = Planner(client, tools={'stop': boom}, schemas=SCHEMAS)
    out = p.run('stop v9')
    assert client.requests[1]['messages'][-1]['content'][0]['is_error'] is True
    assert out.text.startswith('v9')


def test_request_shape_has_fallbacks_and_no_cmd_vel_tool():
    client = StubClient([NS(stop_reason='end_turn', usage=usage(), content=[NS(type='text', text='ok')])])
    Planner(client, tools={}, schemas=SCHEMAS).run('hi')
    req = client.requests[0]
    assert req['fallbacks'] == 'default' and 'server-side-fallback-2026-07-01' in req['betas']
    assert req['model'] == 'claude-opus-5-5'
    assert all('cmd_vel' not in s['name'] and 'cmd_vel' not in s['description'] for s in req['tools'])


def test_sdk_error_keeps_node_alive():
    class Bad(StubClient):
        def create(self, **kwargs):
            raise ConnectionError('offline')
    out = Planner(Bad([]), tools={}, schemas=SCHEMAS).run('hi')
    assert out.error.startswith('ConnectionError') and not out.refused


def test_rich_tool_content_passes_blocks_and_trims_trace():
    blocks = [{'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/png', 'data': 'AAAA'}},
              {'type': 'text', 'text': 'camera v1'}]
    client = StubClient([
        NS(stop_reason='tool_use', usage=usage(), content=[NS(type='tool_use', id='t1', name='describe_scene', input={'vehicle': 'v1'})]),
        NS(stop_reason='end_turn', usage=usage(), content=[NS(type='text', text='a red box')]),
    ])
    p = Planner(client, tools={'describe_scene': lambda vehicle: {'vehicle': vehicle, 'width': 640, '_content': blocks}}, schemas=SCHEMAS)
    p.run('what does v1 see')
    sent = client.requests[1]['messages'][-1]['content'][0]['content']
    assert sent == blocks
    assert p.trace[-2]['result'] == {'vehicle': 'v1', 'width': 640, 'content_blocks': ['image', 'text']}
    assert 'AAAA' not in json.dumps(p.trace)
