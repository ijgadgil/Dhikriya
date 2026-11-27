import os

from setuptools import find_packages, setup

PKG = 'cogndn_eval'


def tree(d):
    out = []
    for root, _, files in os.walk(d):
        if files:
            out.append((os.path.join('share', PKG, root), [os.path.join(root, f) for f in files]))
    return out


data = [('share/ament_index/resource_index/packages', ['resource/' + PKG]), ('share/' + PKG, ['package.xml'])]
for d in []:
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
    description='Headless harness, experiments E1-E6, statistics and figures; run analysis tools.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['experiments = cogndn_eval.experiments:main', 'export_schedule = cogndn_eval.export_schedule:main', 'analyze_run = cogndn_eval.analyze_run:main', 'make_replay = cogndn_eval.make_replay:main']},
)
