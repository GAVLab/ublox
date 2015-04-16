## Project Setup
cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

IF(DEFINED BUILD_NO_CLANG)
  option(BUILD_NO_CLANG "Do not use the Clang compiler." OFF)
ENDIF(DEFINED BUILD_NO_CLANG)

# Use clang if available
#IF(EXISTS /usr/bin/clang AND NOT BUILD_NO_CLANG)
#  set(CMAKE_CXX_COMPILER /usr/bin/clang++)
#  set(CMAKE_OSX_DEPLOYMENT_TARGET "")
#  set(SERIAL_BUILD_WARNINGS TRUE)
#  set(CMAKE_BUILD_TYPE Debug)
#ENDIF(EXISTS /usr/bin/clang AND NOT BUILD_NO_CLANG)

IF(NOT DEFINED BUILD_SHARED_LIBS)
    OPTION(BUILD_SHARED_LIBS "Build shared libraries." OFF)
ENDIF(NOT DEFINED BUILD_SHARED_LIBS)

# Detect ROS
set(ROS_ROOT $ENV{ROS_ROOT})
set(ROS_ROOT_DEFINED FALSE)
if(DEFINED ROS_ROOT)
  message("ROS detected.")
  set(ROS_ROOT_DEFINED TRUE)
endif(DEFINED ROS_ROOT)

set(BUILD_WITH_ROS ${ROS_ROOT_DEFINED} CACHE BOOL "Build with ROS support.")

if(BUILD_WITH_ROS AND NOT ROS_ROOT_DEFINED)
  message(FATAL_ERROR "ROS support is selected, but ROS_ROOT is not set.")
endif(BUILD_WITH_ROS AND NOT ROS_ROOT_DEFINED)

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(BUILD_WITH_ROS)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  set(ROS_BUILD_TYPE RelWithDebInfo CACHE STRING "Built type for ROS")
  rosbuild_init()

  #uncomment if you have defined messages
  #rosbuild_genmsg()
  #uncomment if you have defined services
  #rosbuild_gensrv()
else(BUILD_WITH_ROS)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Built type for cmake")
  # Find serial, if it hasn't already been found
  
  if(NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)
    find_package(serial REQUIRED)
  else (NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)
    set(serial_FOUND TRUE)
  endif(NOT DEFINED serial_LIBRARIES OR NOT DEFINED serial_INCLUDE_DIRS)


  if(serial_FOUND)
    include_directories(${serial_INCLUDE_DIRS})
    list(APPEND UBLOX_LINK_LIBS ${serial_LIBRARIES})
    if(UNIX AND NOT APPLE)
        list(APPEND UBLOX_LINK_LIBS rt)
    endif(UNIX AND NOT APPLE)
  else(serial_FOUND)
    message(FATAL_ERROR "Serial library was not found.")
  endif(serial_FOUND)
endif(BUILD_WITH_ROS)

# Add the include folder to the include path
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add default source files
set(UBLOX_SRCS src/ublox.cpp)
# Add default header files
set(UBLOX_HEADERS include/ublox/ublox.h
                  include/ublox/ublox_structures.h)

# Find Boost, if it hasn't already been found
#set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS thread system REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

list(APPEND UBLOX_LINK_LIBS ${Boost_SYSTEM_LIBRARY}
                           ${Boost_THREAD_LIBRARY})

## Build the ublox Library

# Compile the Library
if(BUILD_WITH_ROS)
    rosbuild_add_library(ublox ${UBLOX_SRCS} ${UBLOX_HEADERS})
else(BUILD_WITH_ROS)
    add_library(ublox ${UBLOX_SRCS} ${UBLOX_HEADERS})
endif(BUILD_WITH_ROS)
target_link_libraries(ublox ${UBLOX_LINK_LIBS})

## Build the ROS wrapper

if(BUILD_WITH_ROS)
  rosbuild_add_executable(ublox_node src/ublox_node.cc)
  target_link_libraries(ublox_node ublox)
endif(BUILD_WITH_ROS)

## Build Examples
# If asked to
option(UBLOX_BUILD_EXAMPLES "Build uBlox examples?" OFF)
IF(UBLOX_BUILD_EXAMPLES)
    # Compile the ublox Test program
    add_executable(ublox_example examples/ublox_example.cpp)
    # Link the Test program to the ublox library
    target_link_libraries(ublox_example ublox ${UBLOX_LINK_LIBS})
    # Compile the ublox Test program
    add_executable(assist_example examples/assist_example.cpp)
    # Link the Test program to the ublox library
    target_link_libraries(assist_example ublox ${UBLOX_LINK_LIBS})
    # Compile the ublox Test program
    add_executable(record_pseudoranges examples/record_pseudoranges.cpp)
    # Link the Test program to the ublox library
    target_link_libraries(record_pseudoranges ublox ${UBLOX_LINK_LIBS})
ENDIF(UBLOX_BUILD_EXAMPLES)

## Build tests

# If asked to
IF(UBLOX_BUILD_TESTS)
    
ENDIF(UBLOX_BUILD_TESTS)

## Setup install and uninstall

# Unless asked not to...
IF(NOT UBLOX_DONT_CONFIGURE_INSTALL AND NOT BUILD_WITH_ROS)
  # Configure make install
  IF(NOT CMAKE_INSTALL_PREFIX)
      SET(CMAKE_INSTALL_PREFIX /usr/local)
  ENDIF(NOT CMAKE_INSTALL_PREFIX)
  
  INSTALL(TARGETS ublox
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
  
  INSTALL(FILES include/ublox/ublox.h include/ublox/ublox_structures.h
          DESTINATION include/ublox)
  
  IF(NOT CMAKE_FIND_INSTALL_PATH)
      set(CMAKE_FIND_INSTALL_PATH ${CMAKE_ROOT})
  ENDIF(NOT CMAKE_FIND_INSTALL_PATH)
  
  INSTALL(FILES Findublox.cmake DESTINATION ${CMAKE_FIND_INSTALL_PATH}/Modules/)
  
  ADD_CUSTOM_TARGET(uninstall @echo uninstall package)
  
  IF (UNIX)
    ADD_CUSTOM_COMMAND(
      COMMENT "uninstall package"
      COMMAND xargs ARGS rm < install_manifest.txt
      
      TARGET  uninstall
    )
  ELSE(UNIX)
    ADD_CUSTOM_COMMAND(
      COMMENT "uninstall only implemented in unix"
      TARGET  uninstall
    )
  ENDIF(UNIX)
ENDIF(NOT UBLOX_DONT_CONFIGURE_INSTALL AND NOT BUILD_WITH_ROS)
