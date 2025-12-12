
---

# Desarrollo de técnicas de coordinación de enjambres de vehículos aéreos autónomos no tripulados

Este proyecto contiene algoritmos de comportamiento para drones, implementados usando **ROS 2** y ejecutados en un entorno de desarrollo basado en **Docker**. 
El proyecto tiene como objetivo modelar y simular el comportamiento de drones en un entorno controlado, con el objetivo de optimizar la navegación, el control y otras tareas autónomas.

Se trabaja con la version [ Kilted](https://docs.ros.org/en/kilted/index.html) de ros2 y se utiliza como base el framework [Aerostak 2](https://aerostack2.github.io/index.html)

---

## Requisitos

Para ejecutar este proyecto, necesitarás tener instalados los siguientes componentes:

* **[Docker](https://www.docker.com/)**: Para crear y ejecutar el contenedor de desarrollo.
* **[WSL2](https://docs.microsoft.com/en-us/windows/wsl/)**: Si estás en Windows, es necesario configurar WSL2 para usar Docker de forma nativa.
* **[ROS 2](https://docs.ros.org/en/humble/)**: Este proyecto está diseñado para funcionar con ROS 2 Kilted (Aerostack2 oficialmente recomienda la versión Humble).


---

## Estructura del Proyecto

```
uam-dros2/
│
├── docker/               # Archivos relacionados con Docker (Dockerfile, entrypoint, etc.)
│   ├── Dockerfile
│   ├── run_docker.sh
│   └── entrypoint.sh
│
├── src/                  # Paquetes ROS 2 (tanto propios como externos)
│   ├── aerostack2/
│   ├── mocap4r2_msg/
│   └── ...
│
├── data/                 # Resultados de simulaciones, logs, métricas, etc.
│   ├── metrics/
│   └── logs/
│
├── build/                # Archivos generados por colcon (no deben ir al repositorio)
│
├── install/              # Archivos generados por colcon (no deben ir al repositorio)
│
├── log/                  # Logs generados por colcon (no deben ir al repositorio)
│
├── .gitignore            # Ignorar archivos no necesarios para el repositorio
├── README.md             # Este archivo
├── rosdep.yaml           # Archivos de dependencias para rosdep
└── CMakeLists.txt        # Configuración del proyecto
```

---

## Instalación y Configuración

### 1. Clonar el Repositorio

Primero, clona el repositorio en tu máquina local:

```bash
git clone https://github.com/fjcg-mx/uami-dros2.git
cd uami_dros2
```

### 2. Construir la Imagen de Docker/Ejecutar el Contenedor Docker

Este proyecto se ejecuta dentro de un contenedor Docker para garantizar que el entorno de desarrollo sea consistente en todas las máquinas. Construye la imagen Docker utilizando el `Dockerfile` incluido en el repositorio.

Primero debes de dar permiso de ejecución al escript:
```bash
sudo chmod +x ./docker/run_docker.sh
```
Puedes ejecutar el contenedor Docker. Este comando te proporcionará un entorno de desarrollo completo con ROS 2 instalado:

```bash
./docker/run_docker.sh
```

### 4. Construir el workspace de ROS 2

Para instalar las dependencias, debes ejecutar el siguiente comando dentro de tu workspace ROS 2 (donde se encuentra el archivo rosdep.yaml):

```bash
rosdep install --from-paths src --ignore-src -r -y
```
Esto hará lo siguiente:
* --ignore-src: Ignora los paquetes de código fuente, ya que esos ya están en el repositorio.
* -r: Resuelve las dependencias recursivamente.
* -y: Confirma la instalación de las dependencias sin pedir confirmación.
* --from-paths src: Le dice a rosdep que instale las dependencias para todos los paquetes en el directorio src/.

Dentro del contenedor, navega a la carpeta de trabajo `src` y usa `colcon` para construir los paquetes ROS 2:

```bash
cd /home/dros2_ws
colcon build --symlink-install
```

Este comando instalará las dependencias y construirá los paquetes definidos en `src`.

### 5. Fuente del Espacio de Trabajo

Una vez que el proceso de construcción termine, asegúrate de "source" el espacio de trabajo para que los paquetes ROS 2 sean accesibles en tu entorno:

```bash
source /home/dros2_ws/install/setup.bash
```

---

## Uso del Proyecto

### Ejecutar Simulaciones

Para ejecutar las simulaciones con los algoritmos de comportamiento de los drones, primero asegúrate de que tu contenedor Docker está en ejecución y que tu espacio de trabajo está "sourceado". Luego puedes lanzar los nodos de ROS 2 usando el siguiente comando (esto depende de los paquetes desarrollados):

```bash
ros2 launch my_drone_control my_simulation.launch.py
```

### Ver Métricas y Logs

Durante la ejecución de las simulaciones, los resultados de las métricas y los logs se almacenarán en la carpeta `data/metrics` y `data/logs`. Puedes acceder a estos datos para analizar el rendimiento de los algoritmos.

---

## Contribuir

Si deseas contribuir al proyecto, por favor sigue estos pasos:

1. **Fork** el repositorio.
2. Crea una nueva rama para tus cambios:

   ```bash
   git checkout -b mi-nueva-funcionalidad
   ```
3. Realiza tus cambios y realiza un commit:

   ```bash
   git commit -am 'Añadí una nueva funcionalidad'
   ```
4. Haz **push** a tu rama:

   ```bash
   git push origin mi-nueva-funcionalidad
   ```
5. Crea un **pull request** en GitHub.

---

## Licencia

Este proyecto está licenciado bajo [Apache-2.0 license](LICENSE).

---

## Archivos Importantes

* **`docker/Dockerfile`**: Define la imagen Docker para el entorno de desarrollo.
* **`docker/run_docker.sh`**: Script para ejecutar el contenedor Docker.
* **`src/`**: Contiene los paquetes ROS 2 de control de drones, sensores, y otros algoritmos.
* **`data/`**: Carpeta donde se almacenan los logs y métricas de las simulaciones.
* **`CMakeLists.txt`**: Archivo de configuración de CMake para el proyecto ROS 2.

---

## Contacto

Si tienes alguna pregunta o necesitas ayuda, no dudes en abrir un *issue* en el repositorio o contactar directamente con los desarrolladores.

---

### Personalización y Detalles

* **Dependencias de ROS 2**: Dependencia externa específica en el archivo `rosdep.yaml`
* **Simulaciones**: [Principios de Reynolds, Propuestas existentes](./data/Videos%20de%20simulaciones/videos.md)

---