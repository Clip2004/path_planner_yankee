from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'path_planner_yankee'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clip2004',
    maintainer_email='felipe.mercado59@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_dijkstra_node_yankee = path_planner_yankee.path_planner_dijkstra_node_yankee:main',
            'path_planner_Astar_node_yankee = path_planner_yankee.path_planner_Astar_node_yankee:main',
            'path_planner_dijkstra_improved_node_yankee = path_planner_yankee.path_planner_dijkstra_improved_node_yankee:main',
            'path_mux_node_yankee = path_planner_yankee.path_mux_node_yankee:main',
            'goal_manager_node_yankee = path_planner_yankee.goal_manager_node_yankee:main',
        ],
    },
)