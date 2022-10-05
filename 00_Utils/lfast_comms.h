#include <string>
#include <vector>
#pragma once

#define MAX_MSG_BUFF 2048

namespace LFAST
{
    class Message
    {
    public:
        Message(std::string destIdStr="") : DestIdStr(destIdStr) {}
        std::string DestIdStr;
        template <typename T>
        void addArgument(std::string label, T value);
        std::string getMessageStr();
    private:
        std::vector<std::string> argStrings;
    };
}
