#include "../00_Utils/karbon_can_bus.h"
#include "../00_Utils/bash_wrapper.h"
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
void removeNewlines(std::string &s)
{
    int pos;
    if ((pos = s.find('\n')) != std::string::npos)
        s.erase(pos);
}

///
/// Karbon CAN Bus unit tests ///
///

TEST(karbon_can_bus_tests, kcb_cnstrctr1)
{
    int testVal = 4;
    KarbonCANBus kcb(testVal);
    // EXPECT_EQ(kcb.getTestVar(), testVal);
    SUCCEED();
}

///
/// Bash wrapper unit tests ///
///

TEST(bash_wrapper_tests, testBashCommandWithPipes)
{
    BashCommandWrapper bw;
    bw.StdIn = "This is a wacky test.";
    bw.Command = "grep -c 'wacky'";

    bw.execBashCommandWithPipes();
    int found = std::atoi(bw.StdOut.c_str());

    EXPECT_EQ(1, found);
}

