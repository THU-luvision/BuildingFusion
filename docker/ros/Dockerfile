FROM nvidia/opengl:1.2-glvnd-devel-ubuntu16.04 as glvnd

FROM osrf/ros:kinetic-desktop-full

RUN rm -rf /var/lib/apt/lists/*

# CUDA 9.0
# Source from https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/9.0/ubuntu1604/base/Dockerfile

ENV NVARCH x86_64
ENV NVIDIA_REQUIRE_CUDA "cuda>=9.0"
ENV NV_CUDA_CUDART_VERSION 9.0.176-1

ENV NV_ML_REPO_ENABLED 1
ENV NV_ML_REPO_URL https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/${NVARCH}

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates apt-transport-https gnupg-curl && \
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/${NVARCH}/7fa2af80.pub && \
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/${NVARCH} /" > /etc/apt/sources.list.d/cuda.list && \
    if [ ! -z ${NV_ML_REPO_ENABLED} ]; then echo "deb ${NV_ML_REPO_URL} /" > /etc/apt/sources.list.d/nvidia-ml.list; fi && \
    apt-get purge --auto-remove -y gnupg-curl \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 9.0.176

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-9-0=${NV_CUDA_CUDART_VERSION} \
    && ln -s cuda-9.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

# source from https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/9.0/ubuntu1604/runtime/Dockerfile

ENV NV_CUDA_LIB_VERSION 9.0.176-1
ENV NV_NVTX_VERSION 9.0.176-1
ENV NV_LIBNPP_VERSION 9.0.176-1
ENV NV_LIBCUSPARSE_VERSION 9.0.176-1


ENV NV_LIBCUBLAS_PACKAGE_NAME cuda-cublas-9-0

ENV NV_LIBCUBLAS_VERSION 9.0.176-1
ENV NV_LIBCUBLAS_PACKAGE ${NV_LIBCUBLAS_PACKAGE_NAME}=${NV_LIBCUBLAS_VERSION}

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-9-0=${NV_CUDA_LIB_VERSION} \
    cuda-npp-9-0=${NV_LIBNPP_VERSION} \
    cuda-cusparse-9-0=${NV_LIBCUSPARSE_VERSION} \
    ${NV_LIBCUBLAS_PACKAGE} \
    && rm -rf /var/lib/apt/lists/*

# Keep apt from auto upgrading the cublas and nccl packages. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold ${NV_LIBNCCL_PACKAGE_NAME} ${NV_LIBCUBLAS_PACKAGE_NAME}


# source from https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/9.0/ubuntu1604/devel/Dockerfile

ENV NV_CUDA_LIB_VERSION 9.0.176-1
ENV NV_CUDA_CUDART_DEV_VERSION 9.0.176-1
ENV NV_NVML_DEV_VERSION 9.0.176-1
ENV NV_LIBCUSPARSE_DEV_VERSION 9.0.176-1
ENV NV_LIBNPP_DEV_VERSION 9.0.176-1
ENV NV_LIBCUBLAS_DEV_PACKAGE_NAME cuda-cublas-dev-9-0

ENV NV_LIBCUBLAS_DEV_VERSION 9.0.176-1
ENV NV_LIBCUBLAS_DEV_PACKAGE ${NV_LIBCUBLAS_DEV_PACKAGE_NAME}=${NV_LIBCUBLAS_DEV_VERSION}

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-nvml-dev-9-0=${NV_NVML_DEV_VERSION} \
    cuda-command-line-tools-9-0=${NV_CUDA_LIB_VERSION} \
    cuda-npp-dev-9-0=${NV_LIBNPP_DEV_VERSION} \
    cuda-libraries-dev-9-0=${NV_CUDA_LIB_VERSION} \
    cuda-minimal-build-9-0=${NV_CUDA_LIB_VERSION} \
    ${NV_LIBCUBLAS_DEV_PACKAGE} \
    ${NV_LIBNCCL_DEV_PACKAGE} \
    && rm -rf /var/lib/apt/lists/*

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold ${NV_LIBCUBLAS_DEV_PACKAGE_NAME} ${NV_LIBNCCL_DEV_PACKAGE_NAME}

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

# CUDNN 7

ENV NV_CUDNN_PACKAGE_VERSION 7.6.4.38-1
ENV NV_CUDNN_VERSION 7.6.4.38

ENV NV_CUDNN_PACKAGE_NAME libcudnn7
ENV NV_CUDNN_PACKAGE ${NV_CUDNN_PACKAGE_NAME}=${NV_CUDNN_PACKAGE_VERSION}+cuda9.0

RUN apt-get update && apt-get install -y --no-install-recommends \
    ${NV_CUDNN_PACKAGE} \
    ${NV_CUDNN_PACKAGE_DEV} \
    && apt-mark hold ${NV_CUDNN_PACKAGE_NAME} && \
    rm -rf /var/lib/apt/lists/*

ENV NV_CUDNN_PACKAGE_VERSION 7.6.4.38-1
ENV NV_CUDNN_VERSION 7.6.4.38

ENV NV_CUDNN_PACKAGE_NAME libcudnn7
ENV NV_CUDNN_PACKAGE ${NV_CUDNN_PACKAGE_NAME}=${NV_CUDNN_PACKAGE_VERSION}+cuda9.0
ENV NV_CUDNN_PACKAGE_DEV ${NV_CUDNN_PACKAGE_NAME}-dev=${NV_CUDNN_PACKAGE_VERSION}+cuda9.0

ENV CUDNN_VERSION ${NV_CUDNN_VERSION}

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ${NV_CUDNN_PACKAGE} \
    ${NV_CUDNN_PACKAGE_DEV} \
    && apt-mark hold ${NV_CUDNN_PACKAGE_NAME} && \
    rm -rf /var/lib/apt/lists/*
    
# OpenGL
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

# runtime
# ARG glvnd=nvidia/opengl:1.2-glvnd-devel-ubuntu16.04

COPY --from=glvnd /usr/local/lib/x86_64-linux-gnu /usr/local/lib/x86_64-linux-gnu
COPY --from=glvnd /usr/local/lib/i386-linux-gnu /usr/local/lib/i386-linux-gnu

COPY --from=glvnd /usr/local/share/glvnd /usr/local/share/glvnd

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig

ENV LD_LIBRARY_PATH /usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libxau-dev libxau-dev:i386 \
        libxdmcp-dev libxdmcp-dev:i386 \
        libxcb1-dev libxcb1-dev:i386 \
        libxext-dev libxext-dev:i386 \
        libx11-dev libx11-dev:i386 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=glvnd /usr/local/include /usr/local/include
COPY --from=glvnd /usr/local/lib/i386-linux-gnu/pkgconfig /usr/local/lib/i386-linux-gnu/pkgconfig
COPY --from=glvnd /usr/local/lib/x86_64-linux-gnu/pkgconfig /usr/local/lib/x86_64-linux-gnu/pkgconfig

# Basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
         build-essential \
         cmake \
         git \
         curl \
         vim \
         ca-certificates && \
     rm -rf /var/lib/apt/lists/*

# Run this docker with 
# docker run --rm -it --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 luvisionsigma/opengl:ubuntu16.04_cuda9.0 bash


