#include "../00_Utils/lfast_comms.h"
#include <gtest/gtest.h>

// cd build && ctest --output-on-failure .

TEST(lfast_comms_tests, test_No_arg)
{
    LFAST::Message msg("TestMessage");
    
    EXPECT_STREQ(msg.getMessageStr().c_str(), R"({"TestMessage":""})");
}