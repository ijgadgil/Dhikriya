"""Locate configuration in a source checkout or an installed ROS 2 workspace."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def _share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        return None


def sim_config():
    for c in (os.environ.get('COGNDN_SIM_CONFIG'), os.path.join(HERE, '..', '..', 'cogndn_sim', 'config'),
              os.path.join(_share('cogndn_sim') or '/nonexistent', 'config')):
        if c and os.path.isdir(c):
            return os.path.abspath(c)
    raise FileNotFoundError('cogndn_sim/config not found; set COGNDN_SIM_CONFIG')


def scenario_path(s):
    return s if os.path.isfile(s) else os.path.join(sim_config(), 'scenarios', s + '.yaml')


def town_path():
    return os.path.join(sim_config(), 'town.yaml')


def run_dir():
    d = os.environ.get('COGNDN_RUN_DIR') or os.path.expanduser('~/cogndn_logs/latest')
    os.makedirs(d, exist_ok=True)
    return d
