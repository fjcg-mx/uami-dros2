#!/bin/bash

# Obtener UID y GID del host (usar nombres distintos para evitar conflicto)
HOST_UID=$(id -u)
HOST_GID=$(id -g)
HOST_USER=$USER
HOST_GROUP=$(id -gn)

# Nombre de la imagen y contenedor
IMAGE_NAME="r2k-as2:latest "
CONTAINER_NAME="enjambre-dev-v0"

# Directorio actual (workspace)
WORKSPACE_DIR=$(pwd)
WORKSPACE_ROS2_DIR="/home/dros2_ws"

# Crear carpetas necesarias si no existen
mkdir -p build install log

# Opción para limpiar compilaciones anteriores
if [ "$1" == "clean" ]; then
    echo "Limpiando compilaciones anteriores..."
    rm -rf build/* install/* log/*
fi

# Construir imagen si no existe
# Verificar las imágenes disponibles
echo "Verificando las imágenes locales..."
docker images
# Verificar si la imagen micad_ros2 existe
echo "Verificando si la imagen ${IMAGE_NAME} existe..."
image_id=$(docker images -q ${IMAGE_NAME})

if [ -z "$image_id" ]; then
    echo "Imagen ${IMAGE_NAME} no encontrada. Procediendo a construirla..."
    # Construir la imagen localmente
    docker build \
      --build-arg HOST_UID=$HOST_UID \
      --build-arg HOST_GID=$HOST_GID \
      --build-arg HOST_USER=$HOST_USER \
      --build-arg HOST_GROUP=$HOST_GROUP \
      -t ${IMAGE_NAME} \
      docker/
else
    echo "La imagen ${IMAGE_NAME} ya existe. Usando la imagen existente."
fi
# --no-cache

# Ejecutar contenedor sin sobrescribir el usuario
docker run -it --rm \
  --network=host --ipc=host \
  -v ${WORKSPACE_DIR}/src:${WORKSPACE_ROS2_DIR}/src \
  -v ${WORKSPACE_DIR}/data:${WORKSPACE_ROS2_DIR}/data \
  -v ${WORKSPACE_DIR}/build:${WORKSPACE_ROS2_DIR}/build \
  -v ${WORKSPACE_DIR}/install:${WORKSPACE_ROS2_DIR}/install \
  -v ${WORKSPACE_DIR}/log:${WORKSPACE_ROS2_DIR}/log \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  --name ${CONTAINER_NAME} \
  ${IMAGE_NAME}

# Notas: Estado - No funciona 25/11/2025 12:22 a.m.
