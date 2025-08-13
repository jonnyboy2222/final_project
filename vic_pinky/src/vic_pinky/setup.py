from setuptools import find_packages, setup

package_name = 'vic_pinky'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yhyu',
    maintainer_email='49838082+isac1120@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'arcs_video = vic_pinky.arcs_video:main',
            'arcs_follower = vic_pinky.arcs_follower:main',
            'arcs_navigator = vic_pinky.arcs_navigator:main',
            'arcs_commander = vic_pinky.arcs_commander:main',
            'arcs_aggregator = vic_pinky.arcs_aggregator:main',
            'arcs_arrival_sender = vic_pinky.arcs_arrival_sender:main',
            'arcs_distance_publisher = vic_pinky.arcs_distance_publisher:main',
            'arcs_person = vic_pinky.arcs_person:main',
            'arcs_current_position_publisher = vic_pinky.arcs_current_position_publisher:main',
            'arcs_loadcell_publisher = vic_pinky.arcs_loadcell_publisher:main',
            'arcs_avoid = vic_pinky.arcs_avoid:main',
            'arcs_path = vic_pinky.arcs_path:main',
            'arcs_costmap_follower = vic_pinky.arcs_costmap_follower:main',
            'arcs_simple_follower = vic_pinky.arcs_simple_follower:main',
        ],
    },
)
