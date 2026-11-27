import os

from setuptools import find_packages, setup

PKG = 'cogndn_perception'


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
    description='YOLOv10n detector and LiDAR obstacle clustering nodes.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['yolo_node = cogndn_perception.yolo_node:main', 'lidar_cluster_node = cogndn_perception.lidar_cluster_node:main']},
)
