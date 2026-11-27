import os

from setuptools import find_packages, setup

PKG = 'cogndn_compiler'


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
    description='Cognitive Interest Compiler node (C1) with baseline policies.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['compiler_node = cogndn_compiler.compiler_node:main']},
)
