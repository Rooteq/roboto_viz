from setuptools import find_packages, setup

package_name = 'roboto_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + find_packages(where="./gui_app"),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rooteq',
    maintainer_email='szymerut@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = roboto_viz.publisher:main',
            'gui = gui_app.app:main'
        ],
    },
)
