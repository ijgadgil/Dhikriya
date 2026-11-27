"""Config lookup (re-exports cogndn_core.paths) plus the trust policy file."""
import os

from cogndn_core.paths import _share, scenario_path, sim_config, town_path  # noqa: F401

HERE = os.path.dirname(os.path.abspath(__file__))


def trust_path():
    for c in (os.path.join(HERE, '..', 'config', 'trust.yaml'),
              os.path.join(_share('cogndn_ndn') or '/nonexistent', 'config', 'trust.yaml')):
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError('trust.yaml not found')
