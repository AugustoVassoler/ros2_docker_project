# Usar a imagem base do ROS2 Humble
FROM ros:humble-ros-base

# Instalar dependências para construir pacotes ROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Configurar o diretório de trabalho
WORKDIR /workspace

# Copiar pacotes para o diretório de trabalho
COPY packages/ ./packages/

# Instalar pacotes
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Entrar no ambiente do ROS
ENTRYPOINT ["/bin/bash", "-c", ". /opt/ros/humble/setup.sh && exec bash"]