#include <string>
#include <vector>
#include <iomanip>
#include <limits>
#include <map>
#include <iostream>

#pragma once

#define MAX_RX_DEPTH 5
#define OUTPUT_DEBUG_INFO 0

namespace LFAST
{
void print_map(std::map<std::string, std::string> const &m);
bool isKey(std::string const &);
std::string cleanupKey(std::string const &);
class MessageGenerator
{
    public:
        MessageGenerator() : DestIdStr("") {}
        virtual ~MessageGenerator() {}
        MessageGenerator(std::string destIdStr) : DestIdStr(destIdStr) {}
        std::string DestIdStr;

        template <typename T>
        inline void addArgument(std::string, const T &);
        template <std::size_t N>
        inline void addArgument(std::string, const char (&)[N]);
        inline void addArgument(std::string, MessageGenerator &);
        inline void addArgument(std::string);
        std::string getMessageStr();
        // bool parseMessageStr(std::string const &, unsigned int);

        std::string getArgString(int idx)
        {
            return argStrings.at(idx);
        }

    protected:
        std::vector<std::string> argStrings;
};

struct MessageParser
{

        MessageParser(std::string s)
        {
            this->parseObject(&s);
        }
        typedef std::map<std::string, std::string> RxMessageArg;
        RxMessageArg data;
        MessageParser *child;

        void parseObject(std::string *inBuff);

        bool isNode()
        {
            return child == nullptr;
        }

        bool parseKeyValuePair(std::string *, std::string *, std::string *);
        void printMessage();

    protected:
        std::string parseMessage(std::string *);
};

template <typename T>
inline void MessageGenerator::addArgument(std::string label, const T &value)
{
    std::ostringstream ss;
    ss << std::quoted(label) << ":\"0x" << std::hex << value << "\"";
    this->argStrings.push_back(ss.str());
}
template <std::size_t N>
inline void MessageGenerator::addArgument(std::string label, const char (&value)[N])
{
    std::stringstream ss;
    ss << std::quoted(label) << ":" << std::quoted(std::string(value));
    this->argStrings.push_back(ss.str());
}
template <>
inline void MessageGenerator::addArgument(std::string label, const int &value)
{
    std::ostringstream ss;
    ss << std::quoted(label) << ":" << value;
    this->argStrings.push_back(ss.str());
}
template <>
inline void MessageGenerator::addArgument(std::string label, const bool &value)
{
    std::ostringstream ss;
    ss << std::quoted(label) << ":" << std::boolalpha << value;
    this->argStrings.push_back(ss.str());
}

template <>
inline void MessageGenerator::addArgument(std::string label, const std::string &value)
{
    std::stringstream ss;
    ss << std::quoted(label) << ":" << std::quoted(value);
    this->argStrings.push_back(ss.str());
}

template <>
inline void MessageGenerator::addArgument(std::string label, const double &value)
{
    const int sigdigits = std::numeric_limits<float>::max_digits10;
    std::stringstream ss;
    ss << std::quoted(label) << ":" << std::setprecision(sigdigits) << value;
    this->argStrings.push_back(ss.str());
}

inline void MessageGenerator::addArgument(std::string label, MessageGenerator &msg)
{
    std::stringstream ss;

    ss << std::quoted(label) << ":" << msg.getMessageStr();

    this->argStrings.push_back(ss.str());
}

inline void MessageGenerator::addArgument(std::string label)
{
    std::stringstream ss;
    ss << std::quoted(label) << ":"
       << "null";
    this->argStrings.push_back(ss.str());
}

bool isNumeric(std::string const &str)
{
    long double ld;
    return ((std::istringstream(str) >> ld >> std::ws).eof());
}

// bool isHex(std::string const &str)
// {
//     long double ld;
//     return ((std::istringstream(str) >> ld >> std::ws).eof());
// }

// bool tryGetObjectContents(std::string const &, std::map<std::string, std::string> &);

// std::string tryGetArrayContents(std::string const &);
// std::map<std::string, std::string> tryGetKeyValueMap(std::string const &);
// bool tryGetKeyValueMap(std::string const &str, std::map<std::string, std::string> &);

// template <typename K, typename V>
// void print_map(std::map<K, V> const &m);

bool isObject(std::string str);
}
