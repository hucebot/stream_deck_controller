FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"
LABEL maintainer="clemente.donoso@inria.fr"

WORKDIR /deps

# Install required packages
RUN apt-get update && apt-get upgrade -y && apt-get clean  && apt-get install -y g++-10 python3-dev \
    python-is-python3 \
    python3-pip \
    git \
    ninja-build \
    cmake \
    gedit \
    build-essential \
    libopenblas-dev \
    clang unzip \
    curl \
    wget \
    software-properties-common \
    bash-completion \
    libx11-dev \
    python3-tk \
    neovim \
    libc++-dev \
    libc++abi-dev \
    libomp-dev \
    xorg-dev \
    libxcb-shm0 \
    libglu1-mesa-dev \
    libc++-dev \
    libc++abi-dev \
    libsdl2-dev \
    libxi-dev \
    libtbb-dev \
    libosmesa6-dev \
    libudev-dev \
    autoconf \
    libtool \
    libglew-dev \
    locate\
    nano \
    cmake-curses-gui \
    ffmpeg && \
    apt-get clean
    
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y ros-noetic-desktop-full ros-noetic-srdfdom \
	  ros-noetic-urdf ros-noetic-geometric-shapes ros-noetic-moveit-core ros-noetic-franka-ros ros-noetic-rosmon terminator \
	  ros-noetic-moveit-ros-planning ros-noetic-moveit-ros-planning-interface

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 
# create forest ws and use it to clone and install CONCERT's simulation package
WORKDIR /home/forest_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https
ENV PYTHONUNBUFFERED=1

RUN pip install --upgrade ttictoc hhcm-forest && forest init
RUN echo "source $PWD/setup.bash" >> ~/.bashrc

RUN forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git 

# pre-install pybind11 and custom matlogger2 
RUN forest grow pybind11 --verbose --jobs 4 --pwd user && \
    forest grow matlogger2 --verbose --jobs 4 --pwd user

## HPP-FCL
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/humanoid-path-planner/hpp-fcl.git && \
    ls -a && \
    cd /home/forest_ws/src/hpp-fcl && \
    git checkout 45e60ca7ba81e5394605f8c1097c016245d221c2 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/hpp-fcl && \
    cd /home/forest_ws/build/hpp-fcl && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_PYTHON_INTERFACE=OFF ../../src/hpp-fcl && \
    make -j && \
    make install

## PINOCCHIO
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/stack-of-tasks/pinocchio.git && \
    cd /home/forest_ws/src/pinocchio && \
    git checkout tags/v3.2.0 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/pinocchio && \
    cd /home/forest_ws/build/pinocchio && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_TESTING=FALSE -DBUILD_PYTHON_INTERFACE=OFF ../../src/pinocchio && \
    make -j && \
    make install

# xbot_msgs
WORKDIR /home/forest_ws
RUN source /opt/ros/noetic/setup.bash && forest grow xbot_msgs --verbose --jobs 4 --pwd user

# xbot2_interface
WORKDIR /home/forest_ws/src
RUN git clone -b environment_mesh https://github.com/ADVRHumanoids/xbot2_interface.git && \
    mkdir -p /home/forest_ws/build/xbot2_interface && \
    cd /home/forest_ws/build/xbot2_interface && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DXBOT2_IFC_BUILD_TESTS=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/xbot2_interface && \
    make -j && \
    make install

# osqp
RUN git clone https://github.com/oxfordcontrol/osqp.git && \
    cd /home/forest_ws/src/osqp && \
    git checkout 0b34f2ef5c5eec314e7945762e1c8167e937afbd && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/osqp && \
    cd /home/forest_ws/build/osqp && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DDLONG=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/osqp && \
    make -j && \
    make install

# proxQP
RUN git clone https://github.com/Simple-Robotics/proxsuite.git && \
    cd /home/forest_ws/src/proxsuite && \
    git checkout f19f07b51f66268db1f16cbeb538e891bb6d4e21 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/proxsuite && \
    cd /home/forest_ws/build/proxsuite && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/proxsuite && \
    make -j && \
    make install

# opensot
RUN git clone -b 4.0-devel https://github.com/ADVRHumanoids/OpenSoT.git && \
    mkdir -p /home/forest_ws/build/OpenSoT && \
    cd /home/forest_ws/build/OpenSoT && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DOPENSOT_SOTH_FRONT_END=ON ../../src/OpenSoT && \
    make -j && \
    make install

# reflexxes
RUN git clone https://github.com/ADVRHumanoids/RMLTypeII.git && \
    mkdir -p /home/forest_ws/build/RMLTypeII && \
    cd /home/forest_ws/build/RMLTypeII && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/RMLTypeII && \
    make -j && \
    make install

# CartesI/O
RUN git clone -b local_api_fix https://github.com/ADVRHumanoids/CartesianInterface.git && \
    mkdir -p /home/forest_ws/build/CartesianInterface && cd /home/forest_ws/build/CartesianInterface && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCARTESIO_COMPILE_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/CartesianInterface && \
    make -j && \
    make install

# cartesio_acceleration_support
RUN git clone -b 2.0-devel https://github.com/ADVRHumanoids/cartesio_acceleration_support.git && \
    mkdir -p /home/forest_ws/build/cartesio_acceleration_support && \ 
    cd /home/forest_ws/build/cartesio_acceleration_support && \ 
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCARTESIO_COMPILE_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/cartesio_acceleration_support && \
    make -j && \
    make install

# cartesio_collision_support
RUN git clone -b environment_mesh https://github.com/ADVRHumanoids/cartesio_collision_support.git && \
    mkdir -p /home/forest_ws/build/cartesio_collision_support && \
    cd /home/forest_ws/build/cartesio_collision_support && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCARTESIO_COMPILE_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/cartesio_collision_support && \
    make -j && \
    make install

# base_estimation
RUN git clone -b xbot2ifc https://github.com/ADVRHumanoids/base_estimation.git && \
    mkdir -p /home/forest_ws/build/base_estimation && \
    cd /home/forest_ws/build/base_estimation && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/base_estimation && \
    make -j && \
    make install

## Tiago
RUN git clone -b environment_mesh https://github.com/hucebot/tiago_dual_cartesio_config.git \
    && git clone -b kinetic-devel https://github.com/EnricoMingo/tiago_dual_robot.git \
    && git clone https://github.com/pal-robotics/tiago_dual_description_calibration.git \ 
    && git clone https://github.com/pal-robotics/pal_urdf_utils.git \
    && git clone -b melodic-devel https://github.com/pal-robotics/omni_base_robot.git \
    && git clone -b foxy-devel https://github.com/pal-robotics/tiago_robot.git \
    && git clone -b humble-devel https://github.com/pal-robotics/hey5_description.git \
    && git clone -b humble-devel https://github.com/pal-robotics/pmb2_robot.git \
    && git clone -b humble-devel https://github.com/EnricoMingo/pal_gripper.git

###### Install necessary packages
RUN pip install streamdeck \
    pillow 

###### Install packages for libusb
RUN apt-get install -yy python3-setuptools \
    && apt-get install -yy python3-pip \
    && apt-get install -yy python3-dev \
    && apt-get install -yy libusb-1.0-0-dev \
    && apt-get install -yy libudev-dev \
    && apt-get install -yy libhidapi-libusb0 \
    && apt-get install -yy libjpeg-dev \
    && apt-get install -yy zlib1g-dev \
    && apt-get install -yy libopenjp2-7 \
    && apt-get install -yy libtiff5 \
    && apt-get install -yy openssh-client

###### Install packages for gstreamer
RUN apt-get install -yy h264enc
RUN apt-get install gstreamer1.0-tools
RUN apt-get install -yy libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio 


####### Install package for demo record utils    
RUN apt-get update && apt-get install -y \
    python3-setuptools \
    python3-pip \
    python-is-python3 \
    ffmpeg \
    libsm6 \
    libxext6 \
    ros-noetic-sensor-msgs

RUN pip install rosbags h5py opencv-python cv-bridge tyro

# Compile FFmpeg
RUN sudo apt-get update -qq && sudo apt-get -y install \
    autoconf \
    automake \
    build-essential \
    cmake \
    git-core \
    libass-dev \
    libfreetype6-dev \
    libgnutls28-dev \
    libmp3lame-dev \
    libsdl2-dev \
    libtool \
    libva-dev \
    libvdpau-dev \
    libvorbis-dev \
    libxcb1-dev \
    libxcb-shm0-dev \
    libxcb-xfixes0-dev \
    meson \
    ninja-build \
    pkg-config \
    texinfo \
    wget \
    yasm \
    zlib1g-dev

WORKDIR /ffmpeg_sources
WORKDIR /bin

RUN apt-get update && apt-get install -y software-properties-common \
 && add-apt-repository -y ppa:savoury1/multimedia \
 && apt-get update && apt-get install -y \
    libunistring-dev \
    nasm \
    libx264-dev \
    libx265-dev libnuma-dev \
    libvpx-dev \
    libfdk-aac-dev \
    libopus-dev \
    libdav1d-dev \
 && rm -rf /var/lib/apt/lists/*


# libsvtav1
WORKDIR /ffmpeg_sources
RUN git clone https://gitlab.com/AOMediaCodec/SVT-AV1.git
WORKDIR /ffmpeg_sources/SVT-AV1/build
RUN PATH="$HOME/bin:$PATH" cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX="$HOME/ffmpeg_build" -DCMAKE_BUILD_TYPE=Release -DBUILD_DEC=OFF -DBUILD_SHARED_LIBS=OFF .. && \
    PATH="$HOME/bin:$PATH" make && \ 
    make install

# libaom
WORKDIR /ffmpeg_sources
RUN git clone --depth 1 https://aomedia.googlesource.com/aom
WORKDIR /ffmpeg_sources/aom_build
RUN PATH="$HOME/bin:$PATH" cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX="$HOME/ffmpeg_build" -DENABLE_TESTS=OFF -DENABLE_NASM=on ../aom && \
    PATH="$HOME/bin:$PATH" make && \
    make install

# FFmpeg
WORKDIR /ffmpeg_sources
RUN wget -O ffmpeg-snapshot.tar.bz2 https://ffmpeg.org/releases/ffmpeg-snapshot.tar.bz2 && \
    tar xjvf ffmpeg-snapshot.tar.bz2
WORKDIR /ffmpeg_sources/ffmpeg
RUN PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig" ./configure \
    --prefix="$HOME/ffmpeg_build" \
    --pkg-config-flags="--static" \
    --extra-cflags="-I$HOME/ffmpeg_build/include" \
    --extra-ldflags="-L$HOME/ffmpeg_build/lib" \
    --extra-libs="-lpthread -lm" \
    --ld="g++" \
    --bindir="$HOME/bin" \
    --enable-gpl \
    --enable-gnutls \
    --enable-libaom \
    --enable-libass \
    --enable-libfdk-aac \
    --enable-libfreetype \
    --enable-libmp3lame \
    --enable-libopus \
    --enable-libsvtav1 \
    --enable-libdav1d \
    --enable-libvorbis \
    --enable-libvpx \
    --enable-libx264 \
    --enable-libx265 \
    --enable-nonfree 
RUN PATH="$HOME/bin:$PATH" make && make install
RUN cp ~/bin/ffmpeg /usr/bin/ffmpeg


###### Source the ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN apt-get -y install python-is-python3
RUN apt-get -y install ros-noetic-catkin
RUN pip install --upgrade scipy
RUN pip install tqdm

RUN echo 'export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/home/forest_ws/src/tiago_dual_cartesio_config:/home/forest_ws/src/tiago_dual_robot:/home/forest_ws/src/tiago_dual_description_calibration:/home/forest_ws/src/pal_urdf_utils:/home/forest_ws/src/omni_base_robot:/home/forest_ws/src/tiago_robot:/home/forest_ws/src/hey5_description:/home/forest_ws/src/pmb2_robot:/home/forest_ws/src/pal_gripper"' >> /home/forest_ws/setup.bash