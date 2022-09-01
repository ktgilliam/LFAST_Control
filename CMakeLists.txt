# Define the project name
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(lfast-controller LANGUAGES C CXX VERSION 0.0.1)
cmake_minimum_required(VERSION 3.10)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 1)
set(CDRIVER_VERSION_MINOR 2)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)
message("Generated config.h")

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

message("Enabling testing")
enable_testing()
include(CTest)

add_subdirectory(00_LFAST_Utils)
   
message("Checking for INDI Library")
#Require INDI 1.4 or later
find_package(INDI 1.4 REQUIRED)
if(INDI_FOUND)
   include_directories(${INDI_INCLUDE_DIR})
   add_subdirectory(01_Mount_Driver)

endif(INDI_FOUND)

#find_package(Nova REQUIRED)
#find_package(ZLIB REQUIRED)
#find_package(GSL REQUIRED)




