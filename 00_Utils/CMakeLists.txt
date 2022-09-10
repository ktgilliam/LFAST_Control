########## LFAST Controller Utilities ##############

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

#SET(LFAST_UTILS_SOURCES bash_wrapper.cpp karbon_can_bus.cpp)

#if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
add_subdirectory(test)
#endif (CMAKE_BUILD_TYPE EQUAL "DEBUG")
