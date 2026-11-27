import re
from pathlib import Path


def test_every_launch_policy_has_compiler_layers():
    """Every policy the launch file accepts must be known to the compiler node (else the node crashes)."""
    src = Path(__file__).resolve().parents[2]
    node = (src / 'cogndn_compiler' / 'cogndn_compiler' / 'compiler_node.py').read_text()
    block = node.split('LAYERS = {', 1)[1].split('}\n', 1)[0]
    layers = set(re.findall(r"'(\w+)':", block))
    launch = (src / 'cogndn_bringup' / 'launch' / 'cogndn.launch.py').read_text()
    doc = launch.split('"""')[1]
    policies = set(re.findall(r'\b(cognitive(?:_[a-z]+)?|route_typed|route_all|ondemand)\b', doc))
    assert {'cognitive_vlm', 'cognitive_ocr', 'route_all'} <= policies
    assert policies <= layers, 'missing in compiler LAYERS: %s' % sorted(policies - layers)
