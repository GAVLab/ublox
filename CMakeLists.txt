## Project Setup
cmake_minimum_required(VERSION 2.4.6)

# Use clang if available
  IF(EXISTS /usr/bin/clang)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "")
    set(SERIAL_BUILD_WARNINGS TRUE)
    set(CMAKE_BUILD_TYPE Debug)
  ENDIF(EXISTS /usr/bin/clang)

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

set(UBLOX_LINK_LIBS "")

#if (NOT DEFINED serial_FOUND)
#  SET (serial_FOUND FALSE CACHE BOOL "Has the serial library been found.")
#  SET (serial_INCLUDE_DIRS "C:/serial" CACHE PATH "Serial include directory.")
#  SET (serial_LIBRARIES CACHE FILEPATH "serial.lib")
#endif(NOT DEFINED serial_FOUND)

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
  if(NOT serial_FOUND)
     find_package(serial REQUIRED)
  endif(NOT serial_FOUND)

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
SET (UBLOX_BUILD_EXAMPLES FALSE CACHE BOOL "Build Ublox Examples")
IF(UBLOX_BUILD_EXAMPLES)
    # Compile the ublox Test program
    add_executable(ublox_example examples/ublox_example.cpp)
    # Link the Test program to the ublox library
    target_link_libraries(ublox_example ublox ${UBLOX_LINK_LIBS})

    add_executable(assist_example examples/assist_example.cpp)
    target_link_libraries(assist_example ublox ${UBLOX_LINK_LIBS})

    add_executable(Full_AGPS_assist examples/Full_AGPS_assist.cpp)
    target_link_libraries(Full_AGPS_assist ublox ${UBLOX_LINK_LIBS})
    
    add_executable(almanac_example examples/almanac_example.cpp)
    target_link_libraries(almanac_example ublox ${UBLOX_LINK_LIBS})

    add_executable(HUI_example examples/HUI_example.cpp)
    target_link_libraries(HUI_example ublox ${UBLOX_LINK_LIBS})

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
  
  # INSTALL(FILES Findublox.cmake DESTINATION ${CMAKE_FIND_INSTALL_PATH}/Modules/)
  
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
