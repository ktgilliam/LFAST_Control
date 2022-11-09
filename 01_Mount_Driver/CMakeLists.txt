########## LFAST Mount Driver ##############
# do the replacement in the config.h and LFAST_Mount.xml
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
	${CMAKE_CURRENT_BINARY_DIR}/config.h
)
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/LFAST_Mount.xml.cmake  
	${CMAKE_CURRENT_BINARY_DIR}/LFAST_Mount.xml)

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

# add_library(slew_drive_control slew_drive.cc)
add_executable(LFAST_Mount LFAST_Mount.cc slew_drive.cc)
# add_executable(LFAST_Mount_testable LFAST_Mount.cc)

target_link_libraries(LFAST_Mount   
			 ${INDI_LIBRARIES}
			 ${NOVA_LIBRARIES} 
			 astro_math )

			#  lfast_comms)
			add_compile_definitions(SIM_MODE_ENABLED=1)
# tell cmake where to install our executable
install(TARGETS LFAST_Mount RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/LFAST_Mount.xml
    DESTINATION ${INDI_DATA_DIR}
)
