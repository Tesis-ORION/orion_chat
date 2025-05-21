import glob
import os
from setuptools import find_packages, setup


package_name = 'orion_chat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/sounds', glob.glob('sounds/*.mp3')),
        ('share/' + package_name + '/resource', glob.glob('resource/*.json')),
        ('share/' + package_name + '/orion_chat', glob.glob('orion_chat/*.py')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*.sdf'))


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexoberco',
    maintainer_email='alejandro.bermudez.fajardo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orion_chat = orion_chat.orion_chat:main',  # Nodo de chat principal
            'orion_tts = orion_chat.tts_node:main',  # Nodo de Text-to-Speech
            'orion_stt = orion_chat.stt_node:main',  # Nodo de Speech-to-Text
            'audio_recorder = orion_chat.audio_recorder:main',  # Nodo del grabador de audio
            'audio_player = orion_chat.audio_player:main',  # Nodo del reproductor de audio
        ],
    },
)

