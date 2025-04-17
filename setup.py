import glob
import os
from setuptools import find_packages, setup

def get_data_files(src_folder, install_folder):
    """
    Recorre src_folder y devuelve una lista de tuplas para data_files,
    preservando la estructura de directorios relativa en install_folder.
    """
    data_files = []
    for root, dirs, files in os.walk(src_folder):
        if files:
            # Calcula la ruta relativa respecto a src_folder
            rel_path = os.path.relpath(root, src_folder)
            # Si es la carpeta base, usamos install_folder; si no, concatenamos
            dest = os.path.join(install_folder, rel_path) if rel_path != "." else install_folder
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((dest, file_list))
    return data_files

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

    ] + get_data_files('model', os.path.join('share', package_name, 'model')),
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
            'rosa_controller = orion_chat.rosa_controller:main',  # Nodo de control de ROSA
            'vehicle_agent = orion_chat.vehicle_agent:main',  # Nodo del agente de
            'audio_recorder = orion_chat.audio_recorder:main',  # Nodo del grabador de audio
        ],
    },
)

