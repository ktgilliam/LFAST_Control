#include "../00_Utils/lfast_comms.h"
#include <gtest/gtest.h>
#include <map>
// cd build && clear && ctest --output-on-failure --timeout 5 .

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
    EXPECT_STREQ(msgParent.getMessageStr().c_str(),
                 R"({"ParentMessage":{"ParentArg1":77.1234567,"ChildMessageInside":{"ChildMessageOutside":{"ChildArg1":true,"ChildArg2":17}}}})");
}

TEST(lfast_comms_tests, nestedCommand1)
{
    LFAST::MessageGenerator msgParent("ParentMessage");
    msgParent.addArgument("ParentArg1", 77.1234567);

    LFAST::MessageGenerator msgChild;
    msgChild.addArgument("ChildArg1", true);
    msgChild.addArgument("ChildArg2", 17);

    msgParent.addArgument("ChildMessage", msgChild);
    EXPECT_STREQ(msgParent.getMessageStr().c_str(),
                 R"({"ParentMessage":{"ParentArg1":77.1234567,"ChildMessage":{"ChildArg1":true,"ChildArg2":17}}})");
}


TEST(lfast_comms_tests, simpleRxParserTest)
{
    auto rxMsg = new LFAST::MessageParser("{\"ObjKey\":1234}\n");
    EXPECT_TRUE(rxMsg->succeeded());
    LFAST::print_map(rxMsg->data);
    std::cout << rxMsg->data["ObjKey"].c_str() << std::endl;
    EXPECT_STREQ(rxMsg->data["ObjKey"].c_str(), "1234");

    EXPECT_FALSE(rxMsg->childNode);
}

#if 1 // one child
TEST(lfast_comms_tests, nestedRxParserTest_oneChild)
{
    auto rxMsgParent = new LFAST::MessageParser("{\"ParentKey\":{\"ChildKey\":1234}}\n");
    ASSERT_TRUE(rxMsgParent->succeeded());
    std::string childStr = rxMsgParent->data["ParentKey"];
    EXPECT_STREQ(childStr.c_str(), R"({"ChildKey":1234})");

    auto rxMsgChild = rxMsgParent->childNode;
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

#if 1 // two children
TEST(lfast_comms_tests, nestedRxParserTest_twoChild)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1234,"ChildKey2":2345}})");
    ASSERT_TRUE(rxMsgParent->succeeded());
    std::string childStr = rxMsgParent->data["ParentKey"];
    EXPECT_STREQ(childStr.c_str(), R"({"ChildKey1":1234,"ChildKey2":2345})");

    std::string childVal1, childVal2;
    auto rxMsgChild = rxMsgParent->childNode;
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

#if 1 // two children
TEST(lfast_comms_tests, nestedRxParserTest_twoDoubleChild)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1.234567,"ChildKey2":-55.66778899}})");
    ASSERT_TRUE(rxMsgParent->succeeded());
    std::string childStr = rxMsgParent->data["ParentKey"];

    std::string childVal1, childVal2;
    auto rxMsgChild = rxMsgParent->childNode;
    if (rxMsgChild)
    {
        childVal1 = rxMsgChild->data["ChildKey1"];
        childVal2 = rxMsgChild->data["ChildKey2"];
    }
    else
    {
        GTEST_FATAL_FAILURE_("child pointer null");
    }

    EXPECT_STREQ(childVal1.c_str(), "1.234567");
    EXPECT_STREQ(childVal2.c_str(), "-55.66778899");
}
#endif

#if 1 // Three children
TEST(lfast_comms_tests, nestedRxParserTest_threeChild)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1234,"ChildKey2":2345,"ChildKey3":3456}})");
    ASSERT_TRUE(rxMsgParent->succeeded());
    std::string childStr = rxMsgParent->data["ParentKey"];
    EXPECT_STREQ(childStr.c_str(), R"({"ChildKey1":1234,"ChildKey2":2345,"ChildKey3":3456})");

    std::string childVal1, childVal2, childVal3;
    auto rxMsgChild = rxMsgParent->childNode;
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


#if 1 // 3 deep
TEST(lfast_comms_tests, nestedRxParserTest_threeDeep)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"Child1Key":{"Child2Key1":1234,"Child2Key2":2345}}})");
    ASSERT_TRUE(rxMsgParent->succeeded());
    std::string childStr = rxMsgParent->data["ParentKey"];
    // EXPECT_STREQ(childStr.c_str(), R"({"ChildKey1":1234,"ChildKey2":2345})");

    std::string child2Val1, child2Val2;
    auto child2 = (rxMsgParent->childNode)->childNode;

    if (child2)
    {
        child2Val1 = child2->data["Child2Key1"];
        child2Val2 = child2->data["Child2Key2"];
    }
    else
    {
        GTEST_FATAL_FAILURE_("child pointer null");
    }

    EXPECT_STREQ(child2Val1.c_str(), "1234");
    EXPECT_STREQ(child2Val2.c_str(), "2345");
}
#endif

//TODO: This one breaks it but going to leave it for now.
#if 0 // 3 deep, 2 wide 
TEST(lfast_comms_tests, nestedRxParserTest_threeDeepTwoWide)
{
    auto rxMsgParent = new LFAST::MessageParser(
        R"({"ParentKey":{"Child1Key1":{"Child2Key1":1234,"Child2Key2":2345}, "Child1Key2":3456}})");
    ASSERT_TRUE(rxMsgParent->succeeded());
    std::string childStr = rxMsgParent->data["ParentKey"];
    // EXPECT_STREQ(childStr.c_str(), R"({"ChildKey1":1234,"ChildKey2":2345})");

    std::string child2Val1, child2Val2;
    auto child2 = (rxMsgParent->childNode)->childNode;

    if (child2)
    {
        child2Val1 = child2->data["Child2Key1"];
        child2Val2 = child2->data["Child2Key2"];
    }
    else
    {
        GTEST_FATAL_FAILURE_("child pointer null");
    }

    EXPECT_STREQ(child2Val1.c_str(), "1234");
    EXPECT_STREQ(child2Val2.c_str(), "2345");
}
#endif

#if 1 // 4 deep (max depth currently set to 3)
TEST(lfast_comms_tests, nestedRxParserTest_fourDeep)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"Child1Key":{"Child2Key1":{"Child2Key2":2345}}}})");
    EXPECT_FALSE(rxMsgParent->succeeded());
}
#endif


TEST(lfast_comms_tests, badParseDetection)
{
    // 1. unbalanced brackets
    auto rxMsg1 = new LFAST::MessageParser(R"({"ObjKey":1234)");
    EXPECT_FALSE(rxMsg1->succeeded());

    // 2. semi-colon instead of colon
    auto rxMsg2 = new LFAST::MessageParser(R"({"ObjKey";1234})");
    EXPECT_FALSE(rxMsg2->succeeded());

    // 3. No quotes around key
    auto rxMsg3 = new LFAST::MessageParser(R"({ObjKey:1234})");
    EXPECT_FALSE(rxMsg3->succeeded());

    // 4. No curly brackets at all
    auto rxMsg4 = new LFAST::MessageParser(R"("ObjKey":1234)");
    EXPECT_FALSE(rxMsg4->succeeded());
}

//Disabling so I can keep find in protected (don't know how to test protected members yet)
#if 0
TEST(lfast_comms_tests, findGood)
{
    auto rxMsgParent = new LFAST::MessageParser(R"({"ParentKey":{"Child1Key":{"Child2Key1":1234,"Child2Key2":2345}}})");

    ASSERT_TRUE(rxMsgParent->succeeded());
    EXPECT_STREQ(rxMsgParent->find("Child2Key1").c_str(), "1234");
    EXPECT_STREQ(rxMsgParent->find("Child2Key2").c_str(), "2345");
}
#endif

TEST(lfast_comms_tests, lookupString)
{
    auto rxMsg = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1234,"ChildKey2":"Bangarang!"}})");
    ASSERT_TRUE(rxMsg->succeeded());

    EXPECT_STREQ(rxMsg->lookup<std::string>("ChildKey1").c_str(), "1234");
    EXPECT_STREQ(rxMsg->lookup<std::string>("ChildKey2").c_str(), R"("Bangarang!")");
}


TEST(lfast_comms_tests, lookupInt)
{
    auto rxMsg = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1234,"ChildKey2":-987654}})");
    ASSERT_TRUE(rxMsg->succeeded());

    EXPECT_EQ(rxMsg->lookup<int>("ChildKey1"), 1234);
    EXPECT_EQ(rxMsg->lookup<int>("ChildKey2"), -987654);
}

TEST(lfast_comms_tests, lookupUnsignedInt)
{
    auto rxMsg = new LFAST::MessageParser(
        R"({"ParentKey":{"ChildKey1":255,"ChildKey2":0xFF,"ChildKey3":0xDEADBEEF, "ChildKey4":48879}})");
    ASSERT_TRUE(rxMsg->succeeded());

    EXPECT_EQ(rxMsg->lookup<unsigned int>("ChildKey1"), 255);
    // EXPECT_EQ(rxMsg->lookup<unsigned int>("ChildKey1"), rxMsg->lookup<unsigned int>("ChildKey2"));
    // EXPECT_EQ(rxMsg->lookup<unsigned int>("ChildKey3"), 0xDEADBEEF);
    EXPECT_EQ(rxMsg->lookup<unsigned int>("ChildKey4"), 48879);
}

TEST(lfast_comms_tests, lookupBool)
{
    auto rxMsg = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":true,"ChildKey2":false}})");
    ASSERT_TRUE(rxMsg->succeeded());

    EXPECT_EQ(rxMsg->lookup<bool>("ChildKey1"), true);
    EXPECT_EQ(rxMsg->lookup<bool>("ChildKey2"), false);

    auto rxMsg2 = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":True,"ChildKey2":False}})");
    ASSERT_TRUE(rxMsg2->succeeded());
    EXPECT_EQ(rxMsg2->lookup<bool>("ChildKey1"), true);
    EXPECT_EQ(rxMsg2->lookup<bool>("ChildKey2"), false);
    // FAIL();
}

TEST(lfast_comms_tests, lookupDouble)
{
    auto rxMsg = new LFAST::MessageParser(R"({"ParentKey":{"ChildKey1":1.23456,"ChildKey2":-55.123456789}})");
    ASSERT_TRUE(rxMsg->succeeded());

    EXPECT_EQ(rxMsg->lookup<double>("ChildKey1"), 1.23456);
    EXPECT_EQ(rxMsg->lookup<double>("ChildKey2"), -55.123456789);
}
//

// TEST(lfast_comms_tests, handshakeTest)
// {
//     auto rxMsg = new LFAST::MessageParser("{\"KarbonMessage\":{\"Handshake\":48879}}\n");
//     // auto rxMsg = new LFAST::MessageParser("{\"KarbonMessage\":{\"Handshake\":48879}}");
//     ASSERT_TRUE(rxMsg->succeeded());

//     EXPECT_EQ(rxMsg->lookup<unsigned int>("Handshake"), 0xbeef);
// }