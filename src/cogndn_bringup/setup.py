import os

from setuptools import find_packages, setup

PKG = 'cogndn_bringup'
data = [('share/ament_index/resource_index/packages', ['resource/' + PKG]), ('share/' + PKG, ['package.xml'])]
for d in ('launch', 'config', 'scripts'):
    data.append((os.path.join('share', PKG, d), [os.path.join(d, f) for f in os.listdir(d)]))

setup(name=PKG, version='1.0.0', packages=find_packages(exclude=['test']), data_files=data,
      install_requires=['setuptools'], zip_safe=False, maintainer='Cognitive-NDN team',
      maintainer_email='team@cogndn.local', description='Launch files, parameters and run scripts for Cognitive-NDN.',
      license='MIT', entry_points={'console_scripts': []})
