# Define the project name
project(lfast-controller C CXX)
cmake_minimum_required(VERSION 3.10)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

#Require INDI 1.4 or later
find_package(INDI 1.4 REQUIRED)
if(INDI_FOUND)
   include_directories(${INDI_INCLUDE_DIR})
   add_subdirectory(01_Mount_Driver)
endif(INDI_FOUND)

#find_package(Nova REQUIRED)
#find_package(ZLIB REQUIRED)
#find_package(GSL REQUIRED)

# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 1)
set(CDRIVER_VERSION_MINOR 2)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)



