from setuptools import setup
from glob import glob
setup(name='cooperative_transport', version='0.1.0',
      packages=['cooperative_transport'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/cooperative_transport']),
                  ('share/cooperative_transport', ['package.xml', 'README.md']),
                  ('share/cooperative_transport/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='maze', maintainer_email='maze@todo.todo',
      description='Manual-grasp path readiness and scheduled two-robot transport',
      license='Apache-2.0',
      entry_points={'console_scripts': ['transport_peer = cooperative_transport.transport_peer:main',
                                          'transport_keys = cooperative_transport.keyboard:main']})
