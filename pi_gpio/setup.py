from setuptools import setup

package_name = 'pi_gpio'

setup(
    name=package_name,
    version='0.8.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='H. Melih Erdogan',
    author_email='h.meliherdogan@gmail.com',
    maintainer='H. Melih Erdogan',
    maintainer_email='h.meliherdogan@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 Action Server to control Raspberry Pi GPIO',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pi_gpio_server = ' + package_name + '.pi_gpio_server:main',
        ],
    },
)
