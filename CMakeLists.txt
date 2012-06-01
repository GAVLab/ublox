## Project Setup
cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(ublox)

## Configurations

# Use clang if available
IF(EXISTS /usr/bin/clang)
  set(CMAKE_CXX_COMPILER /usr/bin/clang++)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "")
  set(CMAKE_CXX_FLAGS "-ferror-limit=5")
  set(CMAKE_BUILD_TYPE Debug)
ENDIF(EXISTS /usr/bin/clang)

option(UBLOX_BUILD_TESTS "Build all of the Ublox tests." OFF)
option(UBLOX_BUILD_EXAMPLES "Build all of the Ublox examples." OFF)

# Allow for building shared libs override
IF(NOT BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS OFF)
ENDIF(NOT BUILD_SHARED_LIBS)

# Set the default path for built executables to the "bin" directory
IF(NOT DEFINED(EXECUTABLE_OUTPUT_PATH))
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ENDIF(NOT DEFINED(EXECUTABLE_OUTPUT_PATH))
# set the default path for built libraries to the "lib" directory
IF(NOT DEFINED(LIBRARY_OUTPUT_PATH))
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF(NOT DEFINED(LIBRARY_OUTPUT_PATH))

## Configure the build system

# Add the include folder to the include path
include_directories(${PROJECT_SOURCE_DIR}/include /usr/include/c++/4.6)

# Add default source files
set(UBLOX_SRCS src/ublox.cpp)
# Add default header files
set(UBLOX_HEADERS include/ublox/ublox.h 
                    include/ublox/ublox_structures.h)

# Find Boost, if it hasn't already been found
IF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)
    find_package(Boost COMPONENTS system filesystem thread REQUIRED)
ENDIF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)

link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

set(UBLOX_LINK_LIBS ${Boost_SYSTEM_LIBRARY}
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_THREAD_LIBRARY})

# Find serial, if it hasn't already been found
IF(NOT serial_FOUND)
    find_package(serial REQUIRED)
ENDIF(NOT serial_FOUND)

if(serial_FOUND)
  include_directories(${serial_INCLUDE_DIRS})
  #list(APPEND UBLOX_LINK_LIBS ${serial_LIBRARIES} pthread rt)
  list(APPEND UBLOX_LINK_LIBS serial pthread rt)
else(serial_FOUND)
  message(FATAL_ERROR "Serial library was not found.")
endif(serial_FOUND)

message(${UBLOX_LINK_LIBS})

# Find string_utils, if it hasn't already been found
#IF(NOT string_utils_FOUND)
#    find_package(string_utils REQUIRED)
#ENDIF(NOT string_utils_FOUND)

#if(string_utils_FOUND)
#  include_directories(${string_utils_INCLUDE_DIRS})
#  list(APPEND UBLOX_LINK_LIBS ${string_utils_LIBRARIES})
#else(string_utils_FOUND)
#  message(FATAL_ERROR "String Utils library was not found.")
#endif(string_utils_FOUND)

## Build the Ublox Library

# Compile the Library
add_library(ublox ${UBLOX_SRCS} ${UBLOX_HEADERS})
target_link_libraries(ublox ${UBLOX_LINK_LIBS})

## Build Examples

# If asked to
IF(UBLOX_BUILD_EXAMPLES)
    # Compile the ublox Test program
    add_executable(ublox_example examples/ublox_example.cpp)
    # Link the Test program to the ublox library
    target_link_libraries(ublox_example ublox ${UBLOX_LINK_LIBS})
ENDIF(UBLOX_BUILD_EXAMPLES)

## Build tests

# If asked to
IF(UBLOX_BUILD_TESTS)
    # Find Google Test
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    # Compile the atrv Test program
    add_executable(ublox_tests tests/ublox_tests.cpp)
    # Link the Test program to the atrv library
    target_link_libraries(ublox_tests ${GTEST_BOTH_LIBRARIES}
                          ublox)

    add_test(AllTestsIntest_ublox ublox_tests)
ENDIF(UBLOX_BUILD_TESTS)

## Setup install and uninstall

# Unless asked not to...
IF(NOT UBLOX_DONT_CONFIGURE_INSTALL)
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
ENDIF(NOT UBLOX_DONT_CONFIGURE_INSTALL)
