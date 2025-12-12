# Usar la imagen oficial de ROS 2 kilted Desktop Full
ARG ROS_DISTRO_ARG=kilted

FROM osrf/ros:${ROS_DISTRO_ARG}-desktop

ARG ROS_DISTRO_ARG

# Crear usuario con el mismo UID y GID que el host (wsl2/Ubuntu 24.04.3 LTS noble)
# Copia el nombre y el ID de usuario del host (usuario de WSL2)
ARG HOST_UID
ARG HOST_GID
ARG HOST_USER
ARG HOST_GROUP

# Variables de entorno
ENV USER=$HOST_USER
ENV UID=$HOST_UID
ENV GROUP=$HOST_GROUP
ENV GID=$HOST_GID

ENV HOSTNAME=ftscorpmx

ENV HOME=/home/${USER}
ENV XDG_RUNTIME_DIR=/run/user/${UID}
ENV DROS2_WS=/home/dros2_ws

RUN echo $HOSTNAME > /etc/hostname

# -------------------------
# Crear usuario host dentro del contenedor
# -------------------------
# Eliminar usuario si está presente con el mismo UID que el local
RUN deluser --remove-home $(id -un $UID); exit 0
# Crea un usuario dentro del contenedor con el mismo UID/GID que el del host.
RUN echo "* Usuario personalizado: $USER"
# Crear el grupo principal del usuario (si no existe)
RUN getent group $GROUP || groupadd --force -g $GID $GROUP
# Crear el usuario con UID/GID y agregar a grupos útiles
RUN useradd --create-home --shell /bin/bash --groups adm,sudo,video --uid $UID --gid $GID $USER
# Permitir sudo sin contraseña
RUN echo "$USER  ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
# Cambiar contraseña del usuario
RUN echo "$USER:ftscorpmx" | /usr/sbin/chpasswd 2> /dev/null || echo ""
# Ajustar permisos de audio
RUN if [[ -d "/dev/snd" ]] ; then chgrp -R adm /dev/snd ; fi
# Ajuste de /run/user/<UID>
RUN mkdir -p /run/user/${UID} && \
    chown ${USER}:${GROUP} /run/user/${UID} && \
    chmod 700 /run/user/${UID}

# Declarar las variables de entorno
ENV GZ_VERSION=$ROS_DISTRO_ARG
ENV ROS_DISTRO=$ROS_DISTRO_ARG

# Instalar dependencias necesarias
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    nano \
    git \
    curl \
    wget \
    build-essential \
    python3-pymap3d \
    python3-pydantic \
    libgeographiclib-dev \
    libncurses-dev python3-jinja2 \
    ros-${ROS_DISTRO}-geographic-msgs \
    && rm -rf /var/lib/apt/lists/*

### Instalar Gazebo
RUN echo "Instalar Gazebo ros-${GZ_VERSION}-ros-gz" && \
    apt-get update && apt-get install -y \
    ros-${GZ_VERSION}-ros-gz && \
    apt-get autoclean -y && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*
### Fin: Instalar Gazebo

# Crear workspace
WORKDIR $DROS2_WS
RUN chown -R $USER:$GROUP $DROS2_WS

# Clean up and finalize image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# -------------------------
# Copiar entrypoint 
# -------------------------
COPY entrypoint.sh $DROS2_WS/entrypoint.sh
RUN chmod +x $DROS2_WS/entrypoint.sh

# ------------------------- 
# Usuario por defecto
# -------------------------
USER $HOST_USER

# -------------------------
# Ejecutar entrypoint al iniciar el contenedor
# -------------------------
ENTRYPOINT ["./entrypoint.sh"]