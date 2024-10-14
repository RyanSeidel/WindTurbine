from setuptools import setup, find_packages

setup(
    name='my_gazebo_lib',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'paho-mqtt',   # MQTT client for Raspberry Pi communication
        'PySide2',     # PySide2 for GUI integration
        # Add other dependencies if needed (e.g., numpy, etc.)
    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='A Python package to control Gazebo and display real-time data using MQTT and PySide2',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/my_gazebo_lib',  # GitHub or project link
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
)
