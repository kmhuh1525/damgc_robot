from glob import glob
from setuptools import setup

setup(name='cooperative_manual', version='0.1.0', packages=['cooperative_manual'],
      data_files=[('share/ament_index/resource_index/packages',['resource/cooperative_manual']),
                  ('share/cooperative_manual',['package.xml','README.md']),
                  ('share/cooperative_manual/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, license='Apache-2.0',
      maintainer='kde', maintainer_email='kde@example.com',
      description='Straight-only paired manual transport with explicit arming and watchdogs',
      tests_require=['pytest'],
      entry_points={'console_scripts':['manual_straight = cooperative_manual.node:main',
                                      'follower_prepare = cooperative_manual.follower_prepare:main',
                                      'manual_straight_keys = cooperative_manual.keyboard:main']})
