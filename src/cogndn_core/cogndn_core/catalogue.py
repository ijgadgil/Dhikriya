"""Catalogue of named data types that a vehicle can request over NDN.

Each type has a nominal size, a freshness period (how long a copy stays valid) and a
criticality weight used by the Cognitive Interest Compiler when ranking requests.
All values are configurable through YAML; the defaults below are documented assumptions.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class DataType:
    name: str
    size_kb: float        # nominal Data size (kB)
    freshness_s: float    # FreshnessPeriod set by the producer (s)
    criticality: float    # weight in [0, 1] used for request priority
    qualified: bool = False  # True if the name carries a qualifier (e.g. signal movement)


DEFAULT_TYPES = {
    'map': DataType('map', 200.0, 3600.0, 0.5),
    'hazard': DataType('hazard', 2.0, 30.0, 1.0),
    'signal': DataType('signal', 1.0, 5.0, 0.9, qualified=True),
    'workzone': DataType('workzone', 20.0, 120.0, 0.8),
    'detour': DataType('detour', 150.0, 300.0, 0.7),
    'crosswalk': DataType('crosswalk', 1.0, 3.0, 1.0),
    'scene': DataType('scene', 50.0, 10.0, 0.3),
}

SIGNAL_MOVEMENTS = ('straight', 'left', 'right')


def load_types(cfg=None):
    """Return {name: DataType}; cfg is an optional dict overriding the defaults."""
    types = dict(DEFAULT_TYPES)
    for name, d in (cfg or {}).items():
        base = types.get(name, DataType(name, 1.0, 10.0, 0.5))
        types[name] = DataType(name, float(d.get('size_kb', base.size_kb)),
                               float(d.get('freshness_s', base.freshness_s)),
                               float(d.get('criticality', base.criticality)),
                               bool(d.get('qualified', base.qualified)))
    return types
