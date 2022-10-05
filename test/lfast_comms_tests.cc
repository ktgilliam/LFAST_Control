#include "../00_Utils/lfast_comms.h"
#include <gtest/gtest.h>

// cd build && ctest --output-on-failure .

TEST(lfast_comms_tests, test_No_arg)
{
    LFAST::Message msg("TestMessage");

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":""})");
}

TEST(lfast_comms_tests, oneIntArgPos)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg", 1137);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":1137}})");
}
TEST(lfast_comms_tests, oneIntArgNeg)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg", -1137);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":-1137}})");
}
TEST(lfast_comms_tests, oneUintArg)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg", 0x1234ABCDU);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":"0x1234abcd"}})");
}

TEST(lfast_comms_tests, oneBoolArgTrue)
{
    LFAST::Message msg("TestMessage");
    bool testVal = true;
    msg.addArgument("TestArg", testVal);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":true}})");
}

TEST(lfast_comms_tests, oneBoolArgFalse)
{
    LFAST::Message msg("TestMessage");
    bool testVal = false;
    msg.addArgument("TestArg", testVal);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":false}})");
}

TEST(lfast_comms_tests, oneStringArg)
{
    LFAST::Message msg("TestMessage");
    std::string argStr("Hello World.");
    msg.addArgument("TestArg", argStr);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":"Hello World."}})");
}

TEST(lfast_comms_tests, oneCharStringArg)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg", "Hello World.");
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":"Hello World."}})");
}

TEST(lfast_comms_tests, oneDoubleArg)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg", 77.1234567);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":77.1234567}})");
}

TEST(lfast_comms_tests, twoDoubleArgs)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg1", 77.1234567);
    msg.addArgument("TestArg2", -99.7654321);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg1":77.1234567,"TestArg2":-99.7654321}})");
}

TEST(lfast_comms_tests, oneDoubleOneBool)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg1", 77.1234567);
    msg.addArgument("TestArg2", true);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg1":77.1234567,"TestArg2":true}})");
}

TEST(lfast_comms_tests, oneDoubleOneBoolOneInt)
{
    LFAST::Message msg("TestMessage");
    msg.addArgument("TestArg1", 77.1234567);
    msg.addArgument("TestArg2", true);
    msg.addArgument("TestArg3", 17);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg1":77.1234567,"TestArg2":true,"TestArg3":17}})");
}

TEST(lfast_comms_tests, nestedCommand0)
{
    LFAST::Message msgParent("ParentMessage");
    msgParent.addArgument("ParentArg1", 77.1234567);

    LFAST::Message msgChild("ChildMessageOutside");
    msgChild.addArgument("ChildArg1", true);
    msgChild.addArgument("ChildArg2", 17);

    msgParent.addArgument("ChildMessageInside", msgChild);
    EXPECT_STREQ(msgParent.getMessageStr().c_str(), R"({"ParentMessage":{"ParentArg1":77.1234567,"ChildMessageInside":{"ChildMessageOutside":{"ChildArg1":true,"ChildArg2":17}}}})");
}

TEST(lfast_comms_tests, nestedCommand1)
{
    LFAST::Message msgParent("ParentMessage");
    msgParent.addArgument("ParentArg1", 77.1234567);

    LFAST::Message msgChild;
    msgChild.addArgument("ChildArg1", true);
    msgChild.addArgument("ChildArg2", 17);

    msgParent.addArgument("ChildMessage", msgChild);
    EXPECT_STREQ(msgParent.getMessageStr().c_str(), R"({"ParentMessage":{"ParentArg1":77.1234567,"ChildMessage":{"ChildArg1":true,"ChildArg2":17}}})");
}