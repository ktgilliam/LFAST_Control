########## LFAST Mount Driver ##############

add_executable(lfast_mount lfast_mount_driver.cpp)

target_link_libraries(lfast_mount indidriver)


