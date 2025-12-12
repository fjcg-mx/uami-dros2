#!/bin/bash
# Script de inicialización del contenedor ROS 2

# Comprobar que ROS_DISTRO esté definido
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS_DISTRO no está definida. Aborting..."
    exit 1
fi

echo "¡Bienvenido a Aerostack2 en ROS2 ${ROS_DISTRO}!"
echo "Contenedor ejecutándose como usuario $(whoami)"
git config --global user.name "Francisco Javier C.G."
git config --global user.email "$USER@gmail.com"

# Abrir bash interactivo
exec /bin/bash

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USER/.bashrc
exec bash --login