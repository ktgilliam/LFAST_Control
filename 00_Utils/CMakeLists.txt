########## LFAST Controller Utilities ##############

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

#=================================================================================================#
#========================================= can-utils =============================================#
#=================================================================================================#
# include(FetchContent)
# FetchContent_Declare(
#   can-utils
#   GIT_REPOSITORY https://github.com/linux-can/can-utils.git
#   GIT_TAG        e602e391a56e681b03506c2f0890eb128192ec3d
# )
# FetchContent_MakeAvailable(can-utils)

#=================================================================================================#
#========================================= project libraries =====================================#
#=================================================================================================#
# add_library(karbon_can_bus SHARED karbon_can_bus.cc)

add_library(bash_wrapper SHARED bash_wrapper.cc)

include_directories(MODBUS_INCLUDE_DIRS)

add_library(kinco_driver KincoDriver.cc)
target_link_libraries(
    kinco_driver
    libmodbus
)

# add_library(can_bus_interface SHARED can_bus_interface.cc)
# target_link_libraries(can_bus_interface bash_wrapper)

add_library(lfast_comms SHARED lfast_comms.cc)
# target_link_libraries(can_bus_interface bash_wrapper)