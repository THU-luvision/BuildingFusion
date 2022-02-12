cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(scn)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set(TORCH_PATH "${PROJECT_SOURCE_DIR}/../../../../../deps/libtorch")

find_package(Torch REQUIRED 
             HINTS ${TORCH_PATH})
             
find_package(CUDA REQUIRED)

find_library(${CUDPP} NAMES cudpp)
find_library(${CUDPP_HASH} NAMES cudpp_hash)
find_library(${EASY_PROFILER} NAMES easy_profiler)

file(GLOB_RECURSE srcs SCN/sparseconvnet_cuda.cpp)
file(GLOB_RECURSE cuda SCN/cuda.cu)

set(CUDA_ARCH_BIN "30 35 50 52 61" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
include(CudaComputeTargetFlags.cmake)                  
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false; -Xcompiler; -fopenmp")
set(CUDA_HOST_COMPILER gcc-5)

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11 -fopenmp")
# set(CMAKE_CXX_FLAGS "-g -O0 -msse2 -msse3 -Wall -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp")

cuda_add_library(SCN_cuda_library STATIC ${cuda} )

add_library(${CMAKE_PROJECT_NAME} SHARED ${srcs} )

target_link_libraries(${CMAKE_PROJECT_NAME}
                      ${CUDA_LIBRARIES}
                      ${TORCH_LIBRARIES}
                      SCN_cuda_library
                      cudpp
                      cudpp_hash
                      easy_profiler
)

include_directories($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SCN>
                    ${TORCH_PATH}/include
                    ${CUDA_INCLUDE_DIRS}
)
