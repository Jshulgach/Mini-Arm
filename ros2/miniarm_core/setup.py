from setuptools import setup, find_packages

package_name = 'miniarm_core'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='Jonathan Shulgach',
    maintainer_email='jshulgac@andrew.cmu.edu',
    description='Python client library for controlling the Mini-Arm 6-DOF robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mini-arm = miniarm_core.__main__:main',
        ],
    },
    python_requires='>=3.8',
)
