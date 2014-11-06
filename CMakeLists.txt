################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

#Name the project
project(iron_dome)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set scl base directory
SET(SCL_BASE_DIR ~/Documents/scl-manips-v2.git/)

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

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR} ${SCL_INC_DIR_BASE} ${TAO_INC_DIR} ${EIGEN_INC_DIR} ${CHAI_INC_DIR} ${SUTIL_INC_DIR} ${TIXML_INC_DIR}) 

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(IRON_DOME_SRC_DIR ./src)

#These are the source files that will be compiled.
SET(ALL_SRC ${IRON_DOME_SRC_DIR}/robot_sim.cpp
            ${IRON_DOME_SRC_DIR}/CIronDomeApp.cpp
            ${IRON_DOME_SRC_DIR}/tasks/CTaskGcIronDome.cpp
            ${IRON_DOME_SRC_DIR}/tasks/data_structs/STaskOpIronDome.cpp
            ${IRON_DOME_SRC_DIR}/tasks/CTaskOpIronDome.cpp
            ${IRON_DOME_SRC_DIR}/IronDomeCallbacks.hpp
	    #${IRON_DOME_SRC_DIR}/robot_sim/robot_sim.cpp
            ${SCL_INC_DIR}/robot/CRobotApp.cpp
            ${SCL_INC_DIR}/graphics/chai/CGraphicsChai.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp)

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
target_link_libraries(iron_dome gomp GL GLU GLEW glut ncurses rt dl)

SET(PROJECTILE_SRC ${IRON_DOME_SRC_DIR}/projectile/projectile_test.cpp
                   ${IRON_DOME_SRC_DIR}/projectile/ProjectileGenerator.cpp
                   ${IRON_DOME_SRC_DIR}/projectile/ProjectileEstimator.cpp
                   ${IRON_DOME_SRC_DIR}/projectile/ProjectileManager.cpp)

add_executable(projectile_test ${PROJECTILE_SRC})
