#include "../karbon_can_bus.h"
#include "../bash_wrapper.h"
#include <gtest/gtest.h>
#include <string>
#include <cstring>
#include <sstream>

// To execute tests:
// cd build && ctest --output-on-failure .
///
/// Support functions ///
///
void chomp(std::string &s)
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
    EXPECT_EQ(kcb.getTestVar(), testVal);
}

///
/// Bash wrapper unit tests ///
///

TEST(bash_wrapper_tests, testExecBashCommand)
{
    std::stringstream ss;
    const std::string bashCmd = "echo ";
    const std::string dblQuote = "\"";
    std::string message = "This is a test.";

    ss << bashCmd << dblQuote << message << dblQuote; // << std::endl;
    std::string bashCommandStr = ss.str();
    ss.str(""); // clear the stringstream

    ss << message << std::endl;
    std::string expectedResponseStr = ss.str();
    ss.str(""); // clear the stringstream

    std::string receivedResponseStr;
    int bashExitCode;
    BashWrapper bw;
    receivedResponseStr = bw.execBashCommand(bashCommandStr, bashExitCode);

    EXPECT_STREQ(expectedResponseStr.c_str(), receivedResponseStr.c_str());
}

TEST(bash_wrapper_tests, testExecPipedBashCommand)
{
    BashWrapper::CommandWithPipes cmd;
    std::string inString = R"(This is a wacky test.)";
    cmd.Command = "grep wacky - ";
    cmd.StdIn = inString;
    cmd.execBashCommandWithPipes();

    std::cout << "RECEIVED REPLY: " << cmd.StdOut << std::endl;
    std::stringstream ss;
    ss << cmd.StdOut;
    std::string outString = ss.str();
    chomp(outString);
    ss.str(""); // clear the stringstream
    EXPECT_STREQ(outString.c_str(), inString.c_str());
}