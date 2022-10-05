#include <string>
#include <vector>
#include <iomanip>
#include <limits>

#pragma once

#define MAX_MSG_BUFF 2048

namespace LFAST
{
    class Message
    {
    public:
        Message() : DestIdStr("") {}
        Message(std::string destIdStr) : DestIdStr(destIdStr) {}
        std::string DestIdStr;

        template <typename T>
        inline void addArgument(std::string label, const T &value);
        template <std::size_t N>
        inline void addArgument(std::string label, const char (&value)[N]);
        inline void addArgument(std::string label, Message &msg);
        inline void addArgument(std::string label);
        std::string getMessageStr();

    private:
        std::vector<std::string> argStrings;
    };

    template <typename T>
    inline void Message::addArgument(std::string label, const T &value)
    {
        std::ostringstream ss;
        ss << std::quoted(label) << ":\"0x" << std::hex << value << "\"";
        this->argStrings.push_back(ss.str());
    }
    template <std::size_t N>
    inline void Message::addArgument(std::string label, const char (&value)[N])
    {
        std::stringstream ss;
        ss << std::quoted(label) << ":" << std::quoted(std::string(value));
        this->argStrings.push_back(ss.str());
    }
    template <>
    inline void Message::addArgument(std::string label, const int &value)
    {
        std::ostringstream ss;
        ss << std::quoted(label) << ":" << value;
        this->argStrings.push_back(ss.str());
    }
    template <>
    inline void Message::addArgument(std::string label, const bool &value)
    {
        std::ostringstream ss;
        ss << std::quoted(label) << ":" << std::boolalpha << value;
        this->argStrings.push_back(ss.str());
    }

    template <>
    inline void LFAST::Message::addArgument(std::string label, const std::string &value)
    {
        std::stringstream ss;
        ss << std::quoted(label) << ":" << std::quoted(value);
        this->argStrings.push_back(ss.str());
    }

    template <>
    inline void LFAST::Message::addArgument(std::string label, const double &value)
    {
        const int sigdigits = std::numeric_limits<float>::max_digits10;
        std::stringstream ss;
        ss << std::quoted(label) << ":" << std::setprecision(sigdigits) << value;
        this->argStrings.push_back(ss.str());
    }

    inline void LFAST::Message::addArgument(std::string label, Message &msg)
    {
        std::stringstream ss;

        ss << std::quoted(label) << ":" << msg.getMessageStr();

        this->argStrings.push_back(ss.str());
    }

    inline void LFAST::Message::addArgument(std::string label)
    {
        std::stringstream ss;
        ss << std::quoted(label) << ":" << "null";
        this->argStrings.push_back(ss.str());
    }
}
