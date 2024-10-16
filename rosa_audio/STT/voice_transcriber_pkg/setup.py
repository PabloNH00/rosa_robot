from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'voice_transcriber_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools',  'numpy', 'pyaudio', 'whisper'],
    zip_safe=True,
    maintainer='pablonh-ubuntu',
    maintainer_email='pablo.nuhernandez@gmail.com',
    description='Voice transcription node using Whisper',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_transcriber = voice_transcriber_pkg.transcriber:main'
        ],
    },
)

