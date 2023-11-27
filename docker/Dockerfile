FROM ros:melodic-perception

ENV EIGEN_VERSION="3.3.3"
ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws

      # set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      # work around for GPG key error, see: https://github.com/osrf/docker_images/issues/697
      apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

RUN   apt-get update && apt-get install -y \
      git \
      cmake \
      libatlas-base-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      ros-${ROS_DISTRO}-desktop-full && \
      rm -rf /var/lib/apt/lists/* && \
      # install eigen
      git clone https://gitlab.com/libeigen/eigen.git && \
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
      mkdir -p $CATKIN_WS/src/GVINS/ && \
      cd $CATKIN_WS/src && \
      # clone gnss_comm repo
      git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git

# Build GVINS
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    printf "source /opt/ros/melodic/setup.bash\
            \nsource /root/catkin_ws/devel/setup.bash \n" >> ~/.bashrc
