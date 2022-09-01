cmake_minimum_required(VERSION 3.10)

# set the project name
project(LFAST C CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

#Require INDI 1.4 or later

find_package(INDI 1.4 REQUIRED)
if(INDI_FOUND)
   include_directories(${INDI_INCLUDE_DIR})
   add_subdirectory(mount)
endif(INDI_FOUND)




