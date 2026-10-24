"""The Claude side of the mission planner, with no ROS in it so it can be tested with a stub client.

`Planner.run(text)` drives a manual tool-use loop on `client.beta.messages.create`. Tools are plain
callables registered by name; each call and each model turn is appended to `trace` (list of dicts) so
the ROS node can publish them. A policy refusal is a normal outcome (`Outcome.refused`), never an
exception: the API returns HTTP 200 with `stop_reason == "refusal"` and we branch on that before
touching `content`.
"""
import json
from dataclasses import dataclass, field

PLANNING_MODEL = 'claude-opus-5-5'
FALLBACK_BETA = 'server-side-fallback-2026-07-01'

SYSTEM = """You are the mission planner of a small fleet of simulated ground vehicles patrolling a
mission corridor. You never drive a vehicle directly; you only send goals through the tools, and a
geofence can veto them. Operators type short requests. Use list_vehicles before acting when you do
not know the fleet. Reply with one or two sentences saying what you did and why; the operator reads
the tool trace separately."""


@dataclass
class Outcome:
    text: str = ''
    refused: bool = False
    category: str = ''
    tool_calls: int = 0
    turns: int = 0
    input_tokens: int = 0
    output_tokens: int = 0
    error: str = ''


@dataclass
class Planner:
    client: object
    tools: dict                     # name -> callable(**input) -> JSON-serialisable
    schemas: list                   # Anthropic tool definitions for the same names
    model: str = PLANNING_MODEL
    effort: str = 'medium'
    max_turns: int = 12
    history_turns: int = 6          # user/assistant pairs kept between requests
    trace: list = field(default_factory=list)
    messages: list = field(default_factory=list)

    def _create(self, messages):
        return self.client.beta.messages.create(
            model=self.model,
            max_tokens=4096,
            system=SYSTEM,
            messages=list(messages),
            tools=self.schemas,
            betas=[FALLBACK_BETA],
            fallbacks='default',
            output_config={'effort': self.effort},
        )

    def _trim(self):
        # Keep whole exchanges only: find the last `history_turns` user messages that start an exchange.
        starts = [i for i, m in enumerate(self.messages) if m['role'] == 'user' and isinstance(m['content'], str)]
        if len(starts) > self.history_turns:
            self.messages = self.messages[starts[-self.history_turns]:]

    def run(self, text):
        out = Outcome()
        self.messages.append({'role': 'user', 'content': text})
        self.trace.append({'event': 'request', 'text': text})
        for _ in range(self.max_turns):
            out.turns += 1
            try:
                resp = self._create(self.messages)
            except Exception as exc:  # the SDK's typed errors; keep the node alive
                out.error = f'{type(exc).__name__}: {exc}'
                self.trace.append({'event': 'error', 'error': out.error})
                self.messages.pop()  # the request that failed; keep history consistent
                return out
            usage = getattr(resp, 'usage', None)
            if usage is not None:
                out.input_tokens += getattr(usage, 'input_tokens', 0) or 0
                out.output_tokens += getattr(usage, 'output_tokens', 0) or 0
            if resp.stop_reason == 'refusal':
                details = getattr(resp, 'stop_details', None)
                out.refused = True
                out.category = (getattr(details, 'category', None) or '') if details else ''
                self.trace.append({'event': 'refusal', 'category': out.category, 'model': getattr(resp, 'model', '')})
                self.messages.pop()
                return out
            self.messages.append({'role': 'assistant', 'content': resp.content})
            texts = [b.text for b in resp.content if getattr(b, 'type', '') == 'text']
            if texts:
                out.text = '\n'.join(texts).strip()
            uses = [b for b in resp.content if getattr(b, 'type', '') == 'tool_use']
            if resp.stop_reason != 'tool_use' or not uses:
                self.trace.append({'event': 'reply', 'text': out.text, 'stop_reason': resp.stop_reason})
                self._trim()
                return out
            results = []
            for use in uses:
                out.tool_calls += 1
                fn = self.tools.get(use.name)
                if fn is None:
                    payload, is_error = {'error': f'unknown tool {use.name}'}, True
                else:
                    try:
                        payload, is_error = fn(**(use.input or {})), False
                    except Exception as exc:
                        payload, is_error = {'error': f'{type(exc).__name__}: {exc}'}, True
                self.trace.append({'event': 'tool', 'name': use.name, 'input': use.input, 'result': payload, 'is_error': is_error})
                results.append({'type': 'tool_result', 'tool_use_id': use.id,
                                'content': json.dumps(payload, default=str), 'is_error': is_error})
            self.messages.append({'role': 'user', 'content': results})
        out.error = 'max_turns reached'
        self.trace.append({'event': 'error', 'error': out.error})
        return out
