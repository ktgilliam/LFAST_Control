########## LFAST ARHS Driver ##############
# do the replacement in the config.h and LFAST_AHRS.xml
include(GNUInstallDirs)


configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
	${CMAKE_CURRENT_BINARY_DIR}/config.h
)
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/indi_lfast_ahrs.xml.cmake  
	${CMAKE_CURRENT_BINARY_DIR}/indi_lfast_ahrs.xml)

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
# include_directories( ${CMAKE_SOURCE_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

message(Source dir: "${CMAKE_SOURCE_DIR}")
# find our required packages
find_package(INDI REQUIRED)
# find_package(Nova REQUIRED)

if(INDI_FOUND)
include_directories( ${INDI_INCLUDE_DIR})

set(PROJECT_INCLUDES 
	${INDI_LIBRARIES})

include(CMakeCommon)
add_executable(indi_lfast_ahrs LFAST_AHRS.cc)

target_link_libraries( indi_lfast_ahrs   ${PROJECT_INCLUDES} )

target_include_directories(indi_lfast_ahrs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# tell cmake where to install our executable
install(TARGETS indi_lfast_ahrs RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_lfast_ahrs.xml
    DESTINATION ${INDI_DATA_DIR}
)
message(Install Location set to "${INDI_DATA_DIR}")
endif(INDI_FOUND)