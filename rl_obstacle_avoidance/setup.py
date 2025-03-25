from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob("config/*.json")),

        
        # X3 UAV
        (os.path.join('share', package_name, 'models/x3'), 
         glob('models/x3/*.sdf') + glob('models/x3/*.config')),
        *[(os.path.join('share', package_name, 'models/x3/meshes'), 
           glob('models/x3/meshes/*'))],
        *[(os.path.join('share', package_name, 'models/x3/thumbnails'), 
           glob('models/x3/thumbnails/*'))],
        
        # Parrot Bebop 2
        (os.path.join('share', package_name, 'models/parrot_bebop_2'), 
         glob('models/parrot_bebop_2/*.sdf') + glob('models/parrot_bebop_2/*.config')),
        # Copy all individual files in meshes/
        *[(os.path.join('share', package_name, 'models/parrot_bebop_2/meshes'), 
           glob('models/parrot_bebop_2/meshes/*'))],
        # Copy all individual files in thumbnails/
        *[(os.path.join('share', package_name, 'models/parrot_bebop_2/thumbnails'), 
           glob('models/parrot_bebop_2/thumbnails/*'))],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esmit',
    maintainer_email='erikgsmit@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'q_learning = rl_obstacle_avoidance.q_learning:main',
            'q_learning_test = rl_obstacle_avoidance.q_learning_test:main',
        ],
    },
)
