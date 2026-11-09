from setuptools import find_packages, setup


package_name = 'rdfp_gui_codex'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'README.md', 'requirements.txt', 'requirements-lerobot.txt']),
    ],
    # ROS workspace dependency: package.xml. Desktop dependencies: requirements.txt.
    install_requires=['setuptools'],
    python_requires='>=3.10',
    zip_safe=False,
    maintainer='kwlee',
    maintainer_email='kwlee@todo.todo',
    description='RDFP Dataset Manager — MCAP, PostgreSQL and learning dataset GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dataset_manager = rdfp_gui_codex.manager_cmd:main',
        ],
    },
)
