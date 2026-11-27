"""C3 — evidence-linked explanations: claim extraction, grounding and provenance.

A rationale is broken into claims (one per hazard category it asserts). A claim is grounded
when evidence of the same category exists either in perception (detections within a time
window) or in a verified, signed NDN Data item that was in the VLA's prompt.

  G = |grounded claims| / |claims|           (grounding score, 1.0 when no claims)   (C3-1)

Provenance: each cited Data item is re-checked; the stored content must hash to the digest
recorded at verification time, otherwise the citation is flagged as tampered.
"""
import json
import re

from .context import digest

CATEGORIES = {
    'animal': ['livestock', 'cattle', 'cow', 'cows', 'bull', 'animal', 'animals', 'horse', 'sheep', 'deer', 'dog'],
    'person': ['pedestrian', 'pedestrians', 'person', 'people', 'child', 'children', 'crosswalk', 'school'],
    'workzone': ['work zone', 'workzone', 'roadwork', 'roadworks', 'construction', 'lane closed', 'cones', 'barrels'],
    'closure': ['road closed', 'closed road', 'closure', 'blocked', 'detour', 'accident'],
    'signal': ['red light', 'green light', 'traffic light', 'signal', 'amber'],
}
NEGATIONS = {'no', 'not', 'without', 'none', 'never', 'nor'}

COCO_TO_CATEGORY = {
    'cow': 'animal', 'horse': 'animal', 'sheep': 'animal', 'dog': 'animal', 'bird': 'animal', 'bear': 'animal',
    'elephant': 'animal', 'zebra': 'animal', 'giraffe': 'animal', 'cat': 'animal',
    'person': 'person', 'traffic light': 'signal',
}
TYPE_TO_CATEGORY = {'crosswalk': 'person', 'workzone': 'workzone', 'detour': 'closure', 'signal': 'signal'}
HAZARD_ACTIONS = {'slow_down', 'stop', 'lane_change_left', 'lane_change_right', 'turn_left', 'turn_right'}


def canonical(content):
    return json.dumps(content, sort_keys=True, separators=(',', ':')).encode()


def extract_claims(text):
    """Return the set of hazard categories asserted (not negated) in the rationale."""
    t = ' ' + re.sub(r'[^a-z ]+', ' ', (text or '').lower()) + ' '
    words = t.split()
    found = set()
    for cat, terms in CATEGORIES.items():
        for term in terms:
            for m in re.finditer(r'(?<= )' + re.escape(term) + r'(?= )', t):
                before = t[:m.start()].split()[-3:]
                if not any(w in NEGATIONS for w in before):
                    found.add(cat)
    del words
    return found


def evidence_categories(detections, context_items, min_conf=0.4):
    """Categories supported by perception and by verified Data. Returns (perc_set, data_set)."""
    perc = {COCO_TO_CATEGORY[d['class']] for d in detections
            if d.get('class') in COCO_TO_CATEGORY and d.get('confidence', 0.0) >= min_conf}
    data = set()
    for it in context_items:
        if not it.sig_ok:
            continue
        cat = it.content.get('category') if it.type == 'hazard' else TYPE_TO_CATEGORY.get(it.type)
        if cat and it.content.get('status', 'active') != 'clear':
            data.add(cat)
    return perc, data


def ground(rationale, action, detections, context_items):
    claims = extract_claims(rationale)
    perc, data = evidence_categories(detections, context_items)
    support = {c: ('perception' if c in perc else 'data' if c in data else None) for c in sorted(claims)}
    grounded = [c for c, s in support.items() if s]
    score = 1.0 if not claims else len(grounded) / len(claims)
    justified = action not in HAZARD_ACTIONS or bool(grounded) or bool(perc | data)
    return {'claims': sorted(claims), 'support': support, 'grounding': score,
            'unsupported': [c for c, s in support.items() if not s], 'action_justified': justified}


def check_provenance(cited, store):
    """cited: [{'name', 'digest'}]. Returns list of {'name', 'status'} with status
    'ok' | 'missing' | 'tampered' | 'unsigned'."""
    out = []
    for c in cited:
        it = store.get(c['name'])
        if it is None:
            out.append({'name': c['name'], 'status': 'missing'})
        elif not it.sig_ok:
            out.append({'name': c['name'], 'status': 'unsigned'})
        elif digest(canonical(it.content)) != c['digest'] or it.digest != c['digest']:
            out.append({'name': c['name'], 'status': 'tampered'})
        else:
            out.append({'name': c['name'], 'status': 'ok'})
    return out
