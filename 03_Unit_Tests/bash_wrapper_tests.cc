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
/// Bash wrapper unit tests ///
///

TEST(bash_wrapper_tests, testBashCommandWithPipes)
{
    BashCommand bw;
    bw.StdIn = "This is a wacky test.";
    bw.Command = "grep -c 'wacky'";

    bw.execBashCommandWithPipes();
    int found = std::atoi(bw.StdOut.c_str());

    EXPECT_EQ(1, found);
}

TEST(bash_wrapper_tests, testGetNetworkInterfaces)
{
	BashCommand bw;
	bw.Command = R"(ip link show | grep -P '^\d+:' | sed 's/://g' | awk '{print $2}')";
    bw.execBashCommandWithPipes();
    std::cout << "#### " <<  std::endl << bw.StdOut << std::endl;
    SUCCEED();
}

TEST(bash_wrapper_tests, testStrSplit)
{
    std::string testStr = "this;is;a;test";
    std::string delim = ";";
    BashCommand bc;
    std::vector<std::string> tokens = BashCommand::splitByDelimeter(testStr, delim);
    std::cout << "###" << tokens.size() << "Tokens." <<std::endl;
    EXPECT_STREQ(tokens.at(0).c_str(), "this");
    EXPECT_STREQ(tokens.at(1).c_str(), "is");
    EXPECT_STREQ(tokens.at(2).c_str(), "a");
    EXPECT_STREQ(tokens.at(3).c_str(), "test");
}

TEST(bash_wrapper_tests, testStrSplit)
{
	BashCommand bw;
	bw.Command = R"()";
    bw.execBashCommandWithPipes();
    std::cout << "#### " <<  std::endl << bw.StdOut << std::endl;
    SUCCEED();
}