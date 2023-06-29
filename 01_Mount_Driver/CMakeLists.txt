########## LFAST Mount Driver ##############
# do the replacement in the config.h and LFAST_Mount.xml
include(GNUInstallDirs)


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



# find our required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
# find_package(ZLIB REQUIRED)
# find_package(GSL REQUIRED)

set(CMAKE_BUILD_TYPE Debug)


if(INDI_FOUND)
set(PROJECT_INCLUDES 
	${INDI_LIBRARIES}
	${NOVA_LIBRARIES} 
	PID_Controller 
	KincoDriver)

include(CMakeCommon)
# add_library(slew_drive_control slew_drive.cc)
add_executable(LFAST_Mount LFAST_Mount.cc slew_drive.cc)
# add_executable(LFAST_Mount_testable LFAST_Mount.cc)

target_link_libraries( LFAST_Mount   ${PROJECT_INCLUDES} )

# tell cmake where to install our executable
install(TARGETS LFAST_Mount RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/LFAST_Mount.xml
    DESTINATION ${INDI_DATA_DIR}
)
endif(INDI_FOUND)