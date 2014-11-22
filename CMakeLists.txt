################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

#Name the project
project(iron_dome)

#Set the build mode to release by default
#SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set scl base directory
# NOTE: For portability, create a link between ./scl-repo/ and the
#       location of your scl library.
# Ex: `ln -s ~/Documents/scl-manips-v2.git scl-repo`
SET(SCL_BASE_DIR ./scl-repo/)

###(a) Scl controller
SET(SCL_INC_DIR ${SCL_BASE_DIR}src/scl/)
SET(SCL_INC_DIR_BASE ${SCL_BASE_DIR}src/)
SET(TAO_INC_DIR ${SCL_INC_DIR}dynamics/tao/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INC_DIR ${SCL_BASE_DIR}3rdparty/eigen/)

###(c) Chai3D scenegraph
SET(CHAI_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/chai3d/)
ADD_DEFINITIONS(-D_LINUX -DLINUX)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

### (e) scl_tinyxml (parser)
SET(TIXML_INC_DIR ${SCL_BASE_DIR}3rdparty/tinyxml)

### (f) kalman-cpp (state estimator)
SET(KALMAN_INC_DIR ./external/kalman-cpp)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR} ${SCL_INC_DIR_BASE} ${TAO_INC_DIR} ${EIGEN_INC_DIR}
                    ${CHAI_INC_DIR} ${SUTIL_INC_DIR} ${TIXML_INC_DIR} ${KALMAN_INC_DIR})

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp  -Wno-unused-local-typedefs")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(IRON_DOME_SRC_DIR ./src)

SET(KALMAN_SRC ./external/kalman-cpp/kalman.cpp)

#These are the source files that will be compiled.
SET(ALL_SRC ${IRON_DOME_SRC_DIR}/main.cpp
            ${IRON_DOME_SRC_DIR}/IronDomeApp.cpp
            ${IRON_DOME_SRC_DIR}/ostreamlock.cpp
            ${IRON_DOME_SRC_DIR}/projectile/projectile.cpp
            ${SCL_INC_DIR}/graphics/chai/CGraphicsChai.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp
            ${PROJECTILE_SRC}
            ${KALMAN_SRC})

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(iron_dome ${ALL_SRC})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(iron_dome ${SCL_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(iron_dome ${CHAI_LIBRARY})


###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(iron_dome gomp GL GLU GLEW glut ncurses rt dl zmq zmqpp)

#add_executable(projectile_test ${IRON_DOME_SRC_DIR}/projectile/projectile_test.cpp ${PROJECTILE_SRC})
