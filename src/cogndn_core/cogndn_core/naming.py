"""Hierarchical NDN naming for Cognitive-NDN.

Name layout:  /cogndn/<area>/<zone>/<type>[/<qualifier>]
Examples:     /cogndn/town/m3/hazard
              /cogndn/town/x1/signal/left
"""
PREFIX = '/cogndn'
AREA = 'town'


def make_name(zone, dtype, qualifier=None, area=AREA):
    for part in (zone, dtype) + ((qualifier,) if qualifier else ()):
        if not part or '/' in str(part):
            raise ValueError('invalid name component: %r' % (part,))
    name = '%s/%s/%s/%s' % (PREFIX, area, zone, dtype)
    return name + ('/' + qualifier if qualifier else '')


def parse_name(name):
    """Inverse of make_name: returns dict(area, zone, type, qualifier)."""
    parts = [p for p in name.split('/') if p]
    if len(parts) < 4 or '/' + parts[0] != PREFIX:
        raise ValueError('not a Cognitive-NDN name: %s' % name)
    return {'area': parts[1], 'zone': parts[2], 'type': parts[3],
            'qualifier': parts[4] if len(parts) > 4 else None}
