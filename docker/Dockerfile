FROM nvidia/cuda:11.8.0-base-ubuntu18.04

ENV EIGEN_VERSION="3.3.3"
ENV CERES_VERSION="1.12.0"
ENV ROS_DISTRO='melodic'
ENV CATKIN_WS=/root/catkin_ws

      # set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi
      
RUN   apt-get update && apt-get install -y \
      wget \
      curl \
      git \
      cmake \
      libatlas-base-dev \
      libgoogle-glog-dev \ 
      libsuitesparse-dev \
      python3-pip \
      python3-dev

      # Install ROS melodic
RUN   sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN   apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN   apt-get update
ENV   DEBIAN_FRONTEND=noninteractive
RUN   apt-get install -y \
      ros-${ROS_DISTRO}-desktop \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf
ENV   DEBIAN_FRONTEND=
SHELL ["/bin/bash", "-c"] source /opt/ros/${ROS_DISTRO}/setup.bash
RUN   apt-get install -y \
      python-rosdep \
      python-rosinstall \
      python-rosinstall-generator \
      python-wstool \
      build-essential \
      python-catkin-tools
RUN   rosdep init
RUN   rosdep update

# RUN   rm -rf /var/lib/apt/lists/*
      
      # install eigen
RUN   git clone https://gitlab.com/libeigen/eigen.git && \
      cd eigen && \
      git checkout tags/${EIGEN_VERSION} && \
      mkdir build && cd build && \
      cmake .. && make install && \
      cd ../.. && rm -rf eigen && \
      # Build and install Ceres
      git clone https://ceres-solver.googlesource.com/ceres-solver && \
      cd ceres-solver && \
      git checkout tags/${CERES_VERSION} && \
      mkdir build && cd build && \
      cmake .. && \
      make -j$(USE_PROC) install && \
      cd ../.. && rm -rf ceres-solver && \
      # create GVINS directory
      mkdir -p $CATKIN_WS/src/ && \
      cd $CATKIN_WS/src && \
      # clone gnss_comm repo
      git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git && \
      # clone vins_fusion repo
      git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git

# Build GVINS
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    printf "source /opt/ros/${ROS_DISTRO}/setup.bash\
            \nsource /root/catkin_ws/devel/setup.bash \n" >> ~/.bashrc

# Install pytorch with cuda support
RUN   apt-get install -y python3.8
RUN   python3 -m pip install virtualenv
RUN   virtualenv -p /usr/bin/python3.8 ~/.envs/py38
RUN   ~/.envs/py38/bin/python -m pip install --upgrade pip
RUN   ~/.envs/py38/bin/python -m pip install torch torchvision torchaudio \
      --index-url https://download.pytorch.org/whl/cu118
RUN   ~/.envs/py38/bin/python -m pip install numpy opencv-python matplotlib pyyaml rospkg scipy