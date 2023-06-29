########## LFAST Controller Utilities ##############

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${MODBUS_INCLUDE_DIRS} )
#=================================================================================================#
#========================================= project libraries =====================================#
#=================================================================================================#
add_library(bash_wrapper SHARED bash_wrapper.cc)

add_library(KincoDriver SHARED KincoDriver.cc)
target_link_libraries(KincoDriver ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})

# add_library(can_bus_interface SHARED can_bus_interface.cc)
# target_link_libraries(can_bus_interface bash_wrapper)
add_library(lfast_comms SHARED lfast_comms.cc)
add_library(PID_Controller SHARED PID_Controller.cc)
# add_library(astro_math SHARED astro_math.cc)

# target_link_libraries(astro_math ${INDI_LIBRARIES})