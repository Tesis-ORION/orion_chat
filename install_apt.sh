#!/bin/bash

set -e

# Función para preguntar al usuario si quiere setear las variables de entorno
ask_set_env() {
    echo
    echo "Para que ROS2 se comunique con el robot vía Wi-Fi,"
    echo "ambos (PC y robot) deben estar en la misma red"
    echo "y usar las mismas variables de entorno:"
    echo
    echo "  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
    echo "  export ROS_STATIC_PEERS=\"\""
    echo "  export ROS_DOMAIN_ID=42"
    echo
    echo "Podrás unsetearlas o modificarlas más tarde a tu gusto."
    echo

    # Leer respuesta con timeout de 10 segundos
    read -r -p "¿Deseas aplicarlas ahora? [Y/n] " -t 10 response || response="Y"
    response=${response,,}  # a minúsculas

    if [[ "$response" =~ ^(y|yes|"" ) ]]; then
        export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
        export ROS_STATIC_PEERS=""
        export ROS_DOMAIN_ID=42
        echo ">> Variables de entorno aplicadas."
    else
        echo ">> Variables de entorno NO aplicadas."
    fi

    echo
}

# Antes de la instalación, preguntar
ask_set_env

# ------------------------------------------------
# Instalación de dependencias
# ------------------------------------------------
echo "Actualizando repositorios e instalando paquetes..."
sudo apt update
sudo apt install -y portaudio19-dev \
                    libasound2-dev \
                    alsa-utils \
                    ffmpeg \
                    mpg123 \
                    python-is-python3 \
                    python3-pip

echo "Instalación completada."
