########## LFAST Mount Driver ##############
# do the replacement in the config.h and lfast_mount_driver.xml
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
	${CMAKE_CURRENT_BINARY_DIR}/config.h
)
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/lfast_mount_driver.xml.cmake  
	${CMAKE_CURRENT_BINARY_DIR}/lfast_mount_driver.xml)

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

# add_library(slew_drive_control slew_drive.cc)
add_executable(lfast_mount_driver lfast_mount_driver.cc slew_drive.cc)
# add_executable(lfast_mount_driver_testable lfast_mount_driver.cc)

target_link_libraries(lfast_mount_driver   
			 ${INDI_LIBRARIES}
			 ${NOVA_LIBRARIES} )

			#  lfast_comms)
			add_compile_definitions(SIM_MODE_ENABLED=1)
# tell cmake where to install our executable
install(TARGETS lfast_mount_driver RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/lfast_mount_driver.xml
    DESTINATION ${INDI_DATA_DIR}
)
