cmake_minimum_required(VERSION 3.10)

# set the project name
project(LFAST C CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#Require INDI 1.4 or later

find_package(INDI 1.4 REQUIRED)
if(INDI_FOUND)
   include_directories(${INDI_INCLUDE_DIR})
   add_subdirectory(01_Mount_Driver)
endif(INDI_FOUND)




