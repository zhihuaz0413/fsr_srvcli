from setuptools import find_packages, setup

package_name = 'fsr_srvcli'
pyFSRray = 'pyFSRray'
ni_daqmax = 'ni_daqmx'

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
    maintainer='hrg',
    maintainer_email='zz8723@ic.ac.uk',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = fsr_srvcli.service_fsr_function:main',
            'client = fsr_srvcli.client_fsr_function:main',
        ],
    },
)
