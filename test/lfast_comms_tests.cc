#include "../00_Utils/lfast_comms.h"
#include <gtest/gtest.h>
#include <map>
// cd build && clear && ctest --output-on-failure .

TEST(lfast_comms_tests, test_No_arg)
{
    LFAST::MessageGenerator msg("TestMessage");

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":""})");
}

TEST(lfast_comms_tests, oneIntArgPos)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg", 1234);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":1234}})");
}
TEST(lfast_comms_tests, oneIntArgNeg)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg", -1234);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":-1234}})");
}
TEST(lfast_comms_tests, oneUintArg)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg", 0x1234ABCDU);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":"0x1234abcd"}})");
}

TEST(lfast_comms_tests, oneBoolArgTrue)
{
    LFAST::MessageGenerator msg("TestMessage");
    bool testVal = true;
    msg.addArgument("TestArg", testVal);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":true}})");
}

TEST(lfast_comms_tests, oneBoolArgFalse)
{
    LFAST::MessageGenerator msg("TestMessage");
    bool testVal = false;
    msg.addArgument("TestArg", testVal);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":false}})");
}

TEST(lfast_comms_tests, oneStringArg)
{
    LFAST::MessageGenerator msg("TestMessage");
    std::string argStr("Hello World.");
    msg.addArgument("TestArg", argStr);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":"Hello World."}})");
}

TEST(lfast_comms_tests, oneCharStringArg)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg", "Hello World.");
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":"Hello World."}})");
}

TEST(lfast_comms_tests, oneDoubleArg)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg", 77.1234567);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg":77.1234567}})");
}

TEST(lfast_comms_tests, twoDoubleArgs)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg1", 77.1234567);
    msg.addArgument("TestArg2", -99.7654321);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg1":77.1234567,"TestArg2":-99.7654321}})");
}

TEST(lfast_comms_tests, oneDoubleOneBool)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg1", 77.1234567);
    msg.addArgument("TestArg2", true);

    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg1":77.1234567,"TestArg2":true}})");
}

TEST(lfast_comms_tests, oneDoubleOneBoolOneInt)
{
    LFAST::MessageGenerator msg("TestMessage");
    msg.addArgument("TestArg1", 77.1234567);
    msg.addArgument("TestArg2", true);
    msg.addArgument("TestArg3", 17);
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":{"TestArg1":77.1234567,"TestArg2":true,"TestArg3":17}})");
}

TEST(lfast_comms_tests, nestedCommand0)
{
    LFAST::MessageGenerator msgParent("ParentMessage");
    msgParent.addArgument("ParentArg1", 77.1234567);

    LFAST::MessageGenerator msgChild("ChildMessageOutside");
    msgChild.addArgument("ChildArg1", true);
    msgChild.addArgument("ChildArg2", 17);

    msgParent.addArgument("ChildMessageInside", msgChild);
    EXPECT_STREQ(msgParent.getMessageStr().c_str(), R"({"ParentMessage":{"ParentArg1":77.1234567,"ChildMessageInside":{"ChildMessageOutside":{"ChildArg1":true,"ChildArg2":17}}}})");
}

TEST(lfast_comms_tests, nestedCommand1)
{
    LFAST::MessageGenerator msgParent("ParentMessage");
    msgParent.addArgument("ParentArg1", 77.1234567);

    LFAST::MessageGenerator msgChild;
    msgChild.addArgument("ChildArg1", true);
    msgChild.addArgument("ChildArg2", 17);

    msgParent.addArgument("ChildMessage", msgChild);
    EXPECT_STREQ(msgParent.getMessageStr().c_str(), R"({"ParentMessage":{"ParentArg1":77.1234567,"ChildMessage":{"ChildArg1":true,"ChildArg2":17}}})");
}

TEST(lfast_comms_tests, isNumeric_doubleTests)
{
    std::string testStr1("1.2345");
    EXPECT_TRUE(LFAST::isNumeric(testStr1));

    std::string testStr2("12345");
    EXPECT_TRUE(LFAST::isNumeric(testStr2));

    std::string testStr3("1.23a45");
    EXPECT_FALSE(LFAST::isNumeric(testStr3));
}

TEST(lfast_comms_tests, isObjTest)
{
    EXPECT_TRUE(LFAST::isObject(R"({"ObjLabel":{"ObjKey1":1234}})"));
    EXPECT_FALSE(LFAST::isObject(R"("ObjLabel":{"ObjKey1":1234})"));
}

TEST(lfast_comms_tests, simpleRxParserTest)
{
    auto rxMsg = new LFAST::MessageParser(R"({"ObjKey":1234})");
    LFAST::print_map(rxMsg->data);
    std::cout << rxMsg->data["ObjKey"].c_str() << std::endl;
    EXPECT_STREQ(rxMsg->data["ObjKey"].c_str(), "1234");

    EXPECT_FALSE(rxMsg->child);
}

#if 1 // one child
TEST(lfast_comms_tests, nestedRxParserTest_oneChild)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey":1234}})");
    std::string childStr = rxMsgParent->data["ParentKey"];
    EXPECT_STREQ(childStr.c_str(), R"({"ChildKey":1234})");

    auto rxMsgChild = rxMsgParent->child;
    if (rxMsgChild)
    {
        std::string childVal;
        childVal = rxMsgChild->data["ChildKey"];
        EXPECT_STREQ(childVal.c_str(), "1234");
    }
    else
    {
        GTEST_FATAL_FAILURE_("child pointer null");
    }
}
#endif

#if 2 // two children
TEST(lfast_comms_tests, nestedRxParserTest_twoChild)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1234,"ChildKey2":2345}})");
    std::string childStr = rxMsgParent->data["ParentKey"];
    EXPECT_STREQ(childStr.c_str(), R"({"ChildKey1":1234,"ChildKey2":2345})");

    std::string childVal1, childVal2;
    auto rxMsgChild = rxMsgParent->child;
    if (rxMsgChild)
    {
        childVal1 = rxMsgChild->data["ChildKey1"];
        childVal2 = rxMsgChild->data["ChildKey2"];
    }
    else
    {
        GTEST_FATAL_FAILURE_("child pointer null");
    }

    EXPECT_STREQ(childVal1.c_str(), "1234");
    EXPECT_STREQ(childVal2.c_str(), "2345");
}
#endif

#if 1 // three children
TEST(lfast_comms_tests, nestedRxParserTest_threeChild)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1234,"ChildKey2":2345,"ChildKey3":3456}})");
    std::string childStr = rxMsgParent->data["ParentKey"];
    EXPECT_STREQ(childStr.c_str(), R"({"ChildKey1":1234,"ChildKey2":2345,"ChildKey3":3456})");

    std::string childVal1, childVal2, childVal3;
    auto rxMsgChild = rxMsgParent->child;
    if (rxMsgChild)
    {
        childVal1 = rxMsgChild->data["ChildKey1"];
        childVal2 = rxMsgChild->data["ChildKey2"];
        childVal3 = rxMsgChild->data["ChildKey3"];
    }
    else
    {
        GTEST_FATAL_FAILURE_("child pointer null");
    }

    EXPECT_STREQ(childVal1.c_str(), "1234");
    EXPECT_STREQ(childVal2.c_str(), "2345");
    EXPECT_STREQ(childVal3.c_str(), "3456");
}
#endif

// TEST(lfast_comms_tests, objParse)
// {
//     std::map<std::string, std::string> kvMap;
//     if (LFAST::tryGetObjectContents(R"({"ObjLabel":{"ObjKey1":1234}})", kvMap))
//     {
//         auto keyStr = kvMap["ObjLabel"];
//         EXPECT_STREQ(keyStr.c_str(), R"({"ObjKey1":1234})");
//     }
//     else
//     {
//         LFAST::print_map(kvMap);
//         FAIL();
//     }
// }

// TEST(lfast_comms_tests, keyValueParse)
// {
//     std::map<std::string, std::string> kvMap;

//     if (LFAST::tryGetKeyValueMap(R"("KeyStr": "ValueStr")", kvMap))
//     {
//         auto keyStr = kvMap["KeyStr"];
//         EXPECT_STREQ(keyStr.c_str(), "ValueStr");
//     }
//     else
//     {
//         LFAST::print_map(kvMap);
//         FAIL();
//     }
// }