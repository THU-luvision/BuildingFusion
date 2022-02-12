FROM luvisionsigma/opengl:ubuntu16.04_cuda9.0_ros_kinect

WORKDIR /workspace
COPY . .
# RUN bash build.sh

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglew-dev \
    libsuitesparse-dev \
    libsparsehash-dev &&\
    rm -rf /var/lib/apt/lists/*

RUN cd deps/Open3D/build && cmake .. && make -j && make install &&\
    cd ../../Pangolin/build && cmake .. && make -j && make install &&\
    cd ../../Sophus/build && cmake .. && make -j && make install &&\
    cd ../../cnpy/build && cmake .. && make -j && make install &&\
    cd ../../cudpp/build && cmake .. && make -j && make install &&\
    cd ../../easy_profiler/build && cmake .. && make -j && make install

RUN bash build.sh
# RUN bash /opt/ros/kinetic/setup.bash && source devel/setup.bash 