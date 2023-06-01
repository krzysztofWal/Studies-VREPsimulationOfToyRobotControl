cmake_minimum_required(VERSION 3.17)

# requires this file, for more information look: 
# https://docs.conda.io/projects/conda-build/en/latest/resources/compiler-tools.html
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/_conda-cross-script/cross-linux.cmake)

project(VREP-ControlApp)

add_subdirectory(Packages)
add_subdirectory(ControlApp)
 
