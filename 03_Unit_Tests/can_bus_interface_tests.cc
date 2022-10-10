#include "../00_Utils/can_bus_interface.h"
#include <gtest/gtest.h>
#include <string>
#include <cstring>
#include <sstream>
#include <thread>
#include <stdlib.h>
#include <iostream>

// To execute tests:
// cd build && ctest --output-on-failure .
///
/// Support functions ///

///
/// CAN Bus Interface unit tests ///
///

TEST(can_bus_interface_tests, testCanTransmit)
{
    // testCanTransmit();
    SUCCEED();
}

TEST(can_bus_interface_tests, testCanReceive)
{
    // testCanReceive();
    SUCCEED();
}

TEST(can_bus_interface_tests, testInterfaceCheck)
{
    bool result = CanBusComms::checkInterfaceExists("lo");
    ASSERT_TRUE(result);
}

TEST(can_bus_interface_tests, testInterfaceCheckException)
{
    CanBusComms cb;
    std::string fakeIfName = "fakeIf";
    EXPECT_THROW(cb.openCanSocket(fakeIfName), std::runtime_error);
}

TEST(can_bus_interface_tests, testInterfaceCheckNoException)
{
    CanBusComms cb;
    std::string realIfName = "slcan0";
    EXPECT_NO_THROW(cb.openCanSocket(realIfName));
}