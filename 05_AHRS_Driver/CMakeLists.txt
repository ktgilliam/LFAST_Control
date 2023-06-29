########## LFAST ARHS Driver ##############
# do the replacement in the config.h and LFAST_AHRS.xml
include(GNUInstallDirs)


configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
	${CMAKE_CURRENT_BINARY_DIR}/config.h
)
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/lfast_ahrs_driver.xml.cmake  
	${CMAKE_CURRENT_BINARY_DIR}/lfast_ahrs_driver.xml)

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})



# find our required packages
find_package(INDI REQUIRED)
# find_package(Nova REQUIRED)

if(INDI_FOUND)
set(PROJECT_INCLUDES 
	${INDI_LIBRARIES})

include(CMakeCommon)
add_executable(lfast_ahrs_driver LFAST_AHRS.cc)

target_link_libraries( lfast_ahrs_driver   ${PROJECT_INCLUDES} )

# tell cmake where to install our executable
install(TARGETS lfast_ahrs_driver RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/lfast_ahrs_driver.xml
    DESTINATION ${INDI_DATA_DIR}
)
message(Install Location set to "${INDI_DATA_DIR}")
endif(INDI_FOUND)