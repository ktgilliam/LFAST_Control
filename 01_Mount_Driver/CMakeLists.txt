########## LFAST Mount Driver ##############
# do the replacement in the config.h and LFAST_Mount.xml
include(GNUInstallDirs)

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
	${CMAKE_CURRENT_BINARY_DIR}/config.h
)
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/indi_lfast_mount.xml.cmake  
	${CMAKE_CURRENT_BINARY_DIR}/indi_lfast_mount.xml)

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
# include_directories( ${CMAKE_SOURCE_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})


# find our required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
# find_package(ZLIB REQUIRED)
# find_package(GSL REQUIRED)


if(INDI_FOUND)
set(PROJECT_INCLUDES 
	${INDI_LIBRARIES}
	${NOVA_LIBRARIES} 
	PID_Controller 
	KincoDriver)

include(CMakeCommon)
# add_library(slew_drive_control slew_drive.cc)
add_executable(indi_lfast_mount LFAST_Mount.cc slew_drive.cc)
# add_executable(LFAST_Mount_testable LFAST_Mount.cc)

target_link_libraries( indi_lfast_mount   ${PROJECT_INCLUDES} )


############ Installation directives ############

# tell cmake where to install our executable
install(TARGETS indi_lfast_mount RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_lfast_mount.xml
    DESTINATION ${INDI_DATA_DIR}
)
endif(INDI_FOUND)