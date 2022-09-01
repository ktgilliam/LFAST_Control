########## LFAST Controller Utilities Unit Tests##############

# set our include directories to look for header files
include_directories( ${CMAKE_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})


# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)



## Karbon PC CAN Bus tests
add_executable(
  karbon_can_bus_test
  karbon_can_bus_test.cpp
)

target_link_libraries(
  karbon_can_bus_test
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(karbon_can_bus_test)

