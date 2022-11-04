########## LFAST Controller Utilities ##############

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

#=================================================================================================#
#========================================= project libraries =====================================#
#=================================================================================================#
add_library(bash_wrapper SHARED bash_wrapper.cc)

# add_library(can_bus_interface SHARED can_bus_interface.cc)
# target_link_libraries(can_bus_interface bash_wrapper)

add_library(lfast_comms SHARED lfast_comms.cc)