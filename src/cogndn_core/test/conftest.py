import os
import shutil
import socket
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))
SIM_CFG = os.path.abspath(os.path.join(HERE, '..', '..', 'cogndn_sim', 'config'))

from cogndn_core.world import World  # noqa: E402


@pytest.fixture(scope='session')
def world():
    return World.from_yaml(os.path.join(SIM_CFG, 'town.yaml'))


@pytest.fixture(scope='session')
def scen_dir():
    return os.path.join(SIM_CFG, 'scenarios')


@pytest.fixture
def redis_client():
    if not shutil.which('redis-server'):
        pytest.skip('redis-server not installed')
    import redis
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    proc = subprocess.Popen(['redis-server', '--port', str(port), '--save', '', '--appendonly', 'no'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    r = redis.Redis(port=port)
    for _ in range(50):
        try:
            r.ping()
            break
        except Exception:
            time.sleep(0.05)
    yield r
    proc.terminate()
    proc.wait()
