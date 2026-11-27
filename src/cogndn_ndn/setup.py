import os

from setuptools import find_packages, setup

PKG = 'cogndn_ndn'


def tree(d):
    out = []
    for root, _, files in os.walk(d):
        if files:
            out.append((os.path.join('share', PKG, root), [os.path.join(root, f) for f in files]))
    return out


data = [('share/ament_index/resource_index/packages', ['resource/' + PKG]), ('share/' + PKG, ['package.xml'])]
for d in ['config', 'scripts']:
    data += tree(d)

setup(
    name=PKG,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data,
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Cognitive-NDN team',
    maintainer_email='team@cogndn.local',
    description='NDN data plane: keys and trust policy, signed segmented producer, verifying consumer node, fleet and calibration tools.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['consumer_node = cogndn_ndn.consumer_node:main', 'producer = cogndn_ndn.producer:main', 'make_keys = cogndn_ndn.make_keys:main', 'fleet = cogndn_ndn.fleet:main']},
)
