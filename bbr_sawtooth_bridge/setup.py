from setuptools import setup

package_name = 'bbr_sawtooth_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ruffin White',
    author_email='roxfoxpox@gmail.com',
    maintainer='Ruffin White',
    maintainer_email='roxfoxpox@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Sawtooth Bridge interface for BBR',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_py = bbr_sawtooth_bridge.main:main',
        ],
    },
)
