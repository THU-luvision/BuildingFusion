set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(ZLIB REQUIRED)

include_directories(${ZLIB_INCLUDE_DIRS})

add_library(cnpy SHARED "${CMAKE_CURRENT_SOURCE_DIR}/src/cnpy/cnpy.cpp")
target_link_libraries(cnpy ${ZLIB_LIBRARIES})

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sparseconvnet)

add_library( scn SHARED IMPORTED )
set_target_properties( scn PROPERTIES IMPORTED_LOCATION 
        ${CMAKE_CURRENT_SOURCE_DIR}/sparseconvnet/build/libscn.so )

file(GLOB_RECURSE srcs 
        src/*.cpp)

# set(CMAKE_CXX_FLAGS "-g -O0 -msse2 -msse3 -Wall -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0 -fopenmp")
set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11  -fopenmp")

add_library(scn_cpp ${srcs})

target_link_libraries(scn_cpp
                      ${TORCH_LIBRARIES}
                      cnpy
                      scn
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${TORCH_PATH}/include
)

set(SCN_CPP_INCLUDE_PATH 
                ${CMAKE_CURRENT_SOURCE_DIR}/src 
                ${CMAKE_CURRENT_SOURCE_DIR} 
                PARENT_SCOPE)
