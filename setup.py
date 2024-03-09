from setuptools import find_packages, setup

package_name = 'spot_state_decoder'

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
    maintainer='redwan',
    maintainer_email='redwan06me@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_state_decoder_node = spot_state_decoder.spot_state_decoder_node:main'
        ],
    },
)
