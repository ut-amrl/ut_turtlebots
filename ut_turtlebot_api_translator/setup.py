from setuptools import setup
import os
from glob import glob

package_name = 'ut_turtlebot_api_translator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_amrl_dock_translator = ut_turtlebot_api_translator.turtlebot_amrl_dock_translator:main',
            'turtlebot_action_relayer = ut_turtlebot_api_translator.turtlebot_action_relayer:main',
            'ut_turtlebot_msg_relayer = ut_turtlebot_api_translator.turtlebot_msg_relayer:main'
        ],
    },
)
