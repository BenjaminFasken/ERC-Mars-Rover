# filepath: /home/leo/ERC-Mars-Rover/src/gpio_controller/setup.py
# setup.py
from setuptools import setup
import os # Make sure os is imported if used elsewhere
from glob import glob

package_name = 'gpio_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if any
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')), # Remove this line

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo', # Add your details
    maintainer_email='leo@todo.todo', # Add your details
    description='GPIO controller node', # Add description
    license='Apache License 2.0', # Choose appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpio_node = gpio_controller.gpio_node:main', # Verify this line
        ],
    },
)