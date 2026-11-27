import os

from setuptools import find_packages, setup

PKG = 'cogndn_sim'


def tree(d):
    out = []
    for root, _, files in os.walk(d):
        if files:
            out.append((os.path.join('share', PKG, root), [os.path.join(root, f) for f in files]))
    return out


data = [('share/ament_index/resource_index/packages', ['resource/' + PKG]), ('share/' + PKG, ['package.xml'])]
for d in ['config', 'worlds', 'models', 'launch', 'scripts']:
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
    description='Generated town worlds, Prius model, scenarios, bridge config and simulator launch.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['street_bodies_node = cogndn_sim.street_bodies_node:main']},
)
