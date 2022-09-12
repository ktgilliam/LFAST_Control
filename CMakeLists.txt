# Define the project name
# Have to suppress this warning for sanity
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(lfast-controller LANGUAGES C CXX VERSION 0.0.1)
cmake_minimum_required(VERSION 3.10)



# specify the C++ standard
# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#=================================================================================================#
#========================================= Version Info  =========================================#
#=================================================================================================#
# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 1)
set(CDRIVER_VERSION_MINOR 2)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)
message("Generated config.h")

#=================================================================================================#
#========================================= Dependencies  =========================================#
#=================================================================================================#
# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

set(BUILD_TESTS true)


message("Checking for INDI Library")
#Require INDI 1.4 or later
find_package(INDI 1.4 REQUIRED)
if(INDI_FOUND)
   include_directories(${INDI_INCLUDE_DIR})
   add_subdirectory(00_Utils)
   add_subdirectory(01_Mount_Driver)
   if (BUILD_TESTS)
	message("BUILD_TESTS=TRUE, unit tests will be built.")
	enable_testing()
	include(CTest)
	add_subdirectory(test)
   else ()
        message("BUILD_TESTS=FALSE, unit tests will NOT be built.")
   endif ()
endif(INDI_FOUND)

#find_package(Nova REQUIRED)
#find_package(ZLIB REQUIRED)
#find_package(GSL REQUIRED)




