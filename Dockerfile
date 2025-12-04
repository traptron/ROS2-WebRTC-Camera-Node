FROM ros:jazzy

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libavdevice-dev \
    libavfilter-dev \
    libavutil-dev \
    libswscale-dev \
    libswresample-dev \
    libopus-dev \
    libvpx-dev \
    libsrtp2-dev \
    libsdl2-dev \
    libjpeg-dev \
    libtiff5-dev \
    libpng-dev \
    libopenjp2-7-dev \
    libwebp-dev \
    libharfbuzz-dev \
    libfribidi-dev \
    libxcb1-dev \
    libavc1394-dev \
    libdc1394-dev \
    libiec61883-dev \
    libv4l-dev \
    libx264-dev \
    libx265-dev \
    libnuma-dev \
    libass-dev \
    libfreetype6-dev \
    libbluray-dev \
    libmp3lame-dev \
    libopencore-amrnb-dev \
    libopencore-amrwb-dev \
    libopenjp2-7-dev \
    libspeex-dev \
    libtheora-dev \
    libtwolame-dev \
    libvidstab-dev \
    libvo-amrwbenc-dev \
    libvorbis-dev \
    libwavpack-dev \
    libwebp-dev \
    libxvidcore-dev \
    libxv-dev \
    libzmq3-dev \
    libopenexr-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstrtspserver-1.0-dev \
    libglib2.0-dev \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Создание виртуального окружения
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Установка pip, setuptools, wheel и cython
RUN pip install --break-system-packages --upgrade pip setuptools wheel cython

# Установка пакетов с --prefer-binary и конкретными версиями
RUN pip install --break-system-packages --prefer-binary \
    numpy==1.26.4 \
    opencv-python==4.8.1.78 \
    aiohttp \
    transforms3d \
    pyyaml

# Установка aiortc и av с --prefer-binary
RUN pip install --break-system-packages --prefer-binary av==11.0.0
RUN pip install --break-system-packages --prefer-binary aiortc==1.9.0

# Установка ROS2 пакетов
RUN apt-get update && apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Создание рабочей директории
WORKDIR /app

# Копирование файла ноды
COPY webrtc_camera_node.py /app/

# Открытие порта для WebRTC сервера
EXPOSE 8080

# Запуск ноды
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 webrtc_camera_node.py"]
