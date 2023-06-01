# this script is taken from repository 
# https://github.com/AnacondaRecipes/libnetcdf-feedstock/blob/master/recipe/cross-linux.cmake
# and slightly modified to match conda environment
# the CMAKE_FIND_ROOT_PATH should be modified to match your conda environment
# ===========================================================================

# this one is important
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_PLATFORM Linux)
#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
set(CMAKE_CXX_COMPILER $ENV{CXX})
set(CMAKE_C_COMPILER $ENV{C})
set(CMAKE_Fortran_COMPILER $ENV{FC})

# where is the target environment
# set(CMAKE_FIND_ROOT_PATH $ENV{PREFIX} $ENV{BUILD_PREFIX}/$ENV{HOST}/sysroot)
set(CMAKE_FIND_ROOT_PATH /home/virtualm/miniconda3/envs/coderefinery/x86_64-conda-linux-gnu/sysroot/)

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# god-awful hack because it seems to not run correct tests to determine this:
set(__CHAR_UNSIGNED___EXITCODE 1)
