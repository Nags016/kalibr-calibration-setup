FROM osrf/ros:noetic-desktop-full

# Install all Kalibr dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git wget autoconf automake nano \
    python3-dev python3-pip python3-scipy python3-matplotlib \
    ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    doxygen libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
    python3-catkin-tools python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/*

ENV WORKSPACE=/catkin_ws

# Build Kalibr from source
RUN mkdir -p $WORKSPACE/src && \
    cd $WORKSPACE/src && \
    git clone https://github.com/ethz-asl/kalibr.git && \
    cd $WORKSPACE && \
    catkin init && \
    catkin config --extend /opt/ros/noetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build -j2

# Mount point for user data
VOLUME ["/data"]
WORKDIR /data

ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 && exec \"$@\"", "--"]
CMD ["/bin/bash"]
