"""Structured output of the VLA (the Cognitive State) and its JSON schema.

The schema is passed to Ollama's `format` field so that llava-phi3 can only produce a
valid Cognitive State. Zone and type fields are restricted to enumerations built per call,
so the model can only name zones that are actually ahead of the vehicle.
"""
import copy
import json
from typing import List, Literal, Optional

from pydantic import BaseModel, Field, ValidationError

ACTIONS = ('keep_lane', 'slow_down', 'stop', 'lane_change_left', 'lane_change_right',
           'turn_left', 'turn_right', 'go_straight')


class InformationNeed(BaseModel):
    type: str
    zone: str
    qualifier: Optional[str] = None
    confidence: float = Field(ge=0.0, le=1.0)


class CognitiveState(BaseModel):
    # Listed first so that the constrained decoder writes what it reads on road signs before it decides.
    signs: List[str] = Field(default_factory=list, max_length=4)
    action: Literal[ACTIONS]  # type: ignore[valid-type]
    needs: List[InformationNeed] = Field(default_factory=list, max_length=6)
    rationale: str = Field(default='', max_length=240)


def json_schema(zone_ids, type_names, qualifiers=('straight', 'left', 'right')):
    """JSON schema for Ollama with enumerations restricted to the given zones and types."""
    s = copy.deepcopy(CognitiveState.model_json_schema())
    need = s['$defs']['InformationNeed']['properties']
    need['type'] = {'type': 'string', 'enum': list(type_names)}
    need['zone'] = {'type': 'string', 'enum': list(zone_ids)}
    need['qualifier'] = {'anyOf': [{'type': 'string', 'enum': list(qualifiers)}, {'type': 'null'}], 'default': None}
    s['properties']['rationale']['maxLength'] = 240
    # Sign text comes from the camera OCR (signocr), which reads the full-resolution frame. Asking the model to
    # transcribe signs as well made answers long enough to be cut off, so the field is left out of the schema.
    s['properties'].pop('signs', None)
    if 'signs' in s.get('required', []):
        s['required'].remove('signs')
    return s


def parse_state(text_or_dict, zone_ids=None, type_names=None):
    """Validate raw model output. Needs naming unknown zones/types are dropped (and counted).
    Returns (CognitiveState or None, n_dropped, error_message)."""
    try:
        d = json.loads(text_or_dict) if isinstance(text_or_dict, (str, bytes)) else dict(text_or_dict)
    except ValueError as e:
        return None, 0, 'invalid JSON: %s' % e
    if isinstance(d, dict) and isinstance(d.get('signs'), list):
        # free text copied from signs: keep the first 4 entries, 60 characters each, instead of failing
        d['signs'] = [str(x)[:60] for x in d['signs'][:4]]
    if isinstance(d, dict) and isinstance(d.get('rationale'), str):
        d['rationale'] = d['rationale'][:240]
    try:
        st = CognitiveState.model_validate(d)
    except ValidationError as e:
        return None, 0, str(e).splitlines()[0]
    keep, dropped = [], 0
    for n in st.needs:
        if (zone_ids is not None and n.zone not in zone_ids) or (type_names is not None and n.type not in type_names):
            dropped += 1
            continue
        keep.append(n)
    st.needs = keep
    return st, dropped, ''
