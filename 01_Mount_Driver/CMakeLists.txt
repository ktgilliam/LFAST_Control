########## LFAST Mount Driver ##############

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(lfast_mount_driver lfast_mount_driver.cc)

target_link_libraries(lfast_mount_driver indidriver)

# tell cmake where to install our executable
install(TARGETS lfast_mount_driver RUNTIME DESTINATION bin)
