import os
from glob import glob
from setuptools import setup

package_name = 'ros2swarm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'pattern'),
         glob('launch/pattern/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'pattern', 'movement_pattern'),
         glob('launch/pattern/movement_pattern/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'pattern', 'movement_pattern', 'basic'),
         glob('launch/pattern/movement_pattern/basic/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'pattern', 'movement_pattern', 'combined'),
         glob('launch/pattern/movement_pattern/combined/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'pattern', 'voting_pattern', 'basic'),
         glob('launch/pattern/voting_pattern/basic/*.launch.py')),
        # (os.path.join('share', package_name, 'launch', 'pattern', 'voting_pattern', 'combined'),
        #     glob('launch/pattern/voting_pattern/combined/*.launch.py')),

        (os.path.join('share', package_name, 'config', 'burger'), glob('config/burger/*.yaml')),
        (os.path.join('share', package_name, 'config', 'burger', 'movement_pattern'),
         glob('config/burger/movement_pattern/*.yaml')),
        (os.path.join('share', package_name, 'config', 'burger', 'movement_pattern', 'basic'),
         glob('config/burger/movement_pattern/basic/*.yaml')),
        (os.path.join('share', package_name, 'config', 'burger', 'movement_pattern', 'combined'),
         glob('config/burger/movement_pattern/combined/*.yaml')),
        (os.path.join('share', package_name, 'config', 'burger', 'voting_pattern', 'basic'),
         glob('config/burger/voting_pattern/basic/*.yaml')),
        (os.path.join('share', package_name, 'config', 'waffle_pi'), glob('config/waffle_pi/*.yaml')),
        (os.path.join('share', package_name, 'config', 'waffle_pi', 'movement_pattern'),
         glob('config/waffle_pi/movement_pattern/*.yaml')),
        (os.path.join('share', package_name, 'config', 'waffle_pi', 'movement_pattern', 'basic'),
         glob('config/waffle_pi/movement_pattern/basic/*.yaml')),
        (os.path.join('share', package_name, 'config', 'waffle_pi', 'movement_pattern', 'combined'),
         glob('config/waffle_pi/movement_pattern/combined/*.yaml')),
        (os.path.join('share', package_name, 'config', 'waffle_pi', 'voting_pattern', 'basic'),
         glob('config/waffle_pi/voting_pattern/basic/*.yaml')),

        (os.path.join('share', package_name, 'config', 'jackal'), glob('config/jackal/*.yaml')),
        (os.path.join('share', package_name, 'config', 'jackal', 'movement_pattern'),
         glob('config/jackal/movement_pattern/*.yaml')),
        (os.path.join('share', package_name, 'config', 'jackal', 'movement_pattern', 'basic'),
         glob('config/jackal/movement_pattern/basic/*.yaml')),
        (os.path.join('share', package_name, 'config', 'jackal', 'movement_pattern', 'combined'),
         glob('config/jackal/movement_pattern/combined/*.yaml')),
        (os.path.join('share', package_name, 'config', 'jackal', 'voting_pattern', 'basic'),
         glob('config/jackal/voting_pattern/basic/*.yaml')),
        # (os.path.join('share', package_name, 'config', 'voting_pattern', 'combined'),
        # glob('config/voting_pattern/combined/*.yaml')),

        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Marian Begemann',
    author_email='marian.begemann(at)student.uni-luebeck.de',
    maintainer='Tanja Kaiser',
    maintainer_email='kaiser(at)iti.uni-luebeck.de',
    keywords=['ros2 swarm pattern robot behaviour dispersion aggregation'
              ' attraction magnetometer flocking voter model'],
    description='Contains swarm behaviour pattern implementations. '
                'These could be launched individually or '
                'get combined by redirecting their output topic streams.'
                'All patterns have param files to adjust their behaviour.'
                'Movement: Basic: drive, dispersion, attraction, aggregation, magnetometer, minimalist flocking'
                'Combined: discussed dispersion'
                'Voting: Basic: voter model, voter model with wifi dBm limiter, majority rule',
    license='apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # hardware protection
            'hardware_protection_layer = ros2swarm.hardware_protection_layer:main',

            # movement_pattern.basic
            'drive_pattern = ros2swarm.movement_pattern.basic.drive_pattern:main',
            'dispersion_pattern = '
            'ros2swarm.movement_pattern.basic.dispersion_pattern:main',
            'aggregation_pattern = '
            'ros2swarm.movement_pattern.basic.aggregation_pattern:main',
            'attraction_pattern = '
            'ros2swarm.movement_pattern.basic.attraction_pattern:main',
            'attraction_pattern2 = '
            'ros2swarm.movement_pattern.basic.attraction_pattern2:main',
            'magnetometer_pattern = '
            'ros2swarm.movement_pattern.basic.magnetometer_pattern:main',
            'random_walk_pattern = '
            'ros2swarm.movement_pattern.basic.random_walk_pattern:main', 
            'minimalist_flocking_pattern = '
            'ros2swarm.movement_pattern.basic.minimalist_flocking_pattern:main',
            'rat_search_pattern = '
            'ros2swarm.movement_pattern.basic.rat_search_pattern:main',
            # movement_pattern.combined
            'discussed_dispersion_pattern = '
            'ros2swarm.movement_pattern.combined.discussed_dispersion_pattern:main',

            # voting.basic
            'voter_model_pattern = '
            'ros2swarm.voting_pattern.basic.voter_model_pattern:main',
            'voter_model_with_limiter_pattern = '
            'ros2swarm.voting_pattern.basic.voter_model_with_limiter_pattern:main',
            'majority_rule_pattern = '
            'ros2swarm.voting_pattern.basic.majority_rule_pattern:main',
        ],
    },
)
