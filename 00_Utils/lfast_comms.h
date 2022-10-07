#include <string>
#include <vector>
#include <iomanip>
#include <limits>
#include <map>
#include <iostream>

#pragma once

#define MAX_RX_DEPTH 5
#define OUTPUT_DEBUG_INFO 1

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
        const char* getMessageCStr();

        std::string getArgString(int idx)
        {
            return argStrings.at(idx);
        }

    protected:
        std::vector<std::string> argStrings;
};

struct MessageParser
{
    MessageParser() : parsingStatus(ParsingStatus::PARSING_INACTIVE) {}
    MessageParser(std::string s)
    {
        this->parseMessageBuffer(&s);
    }

    enum class ParsingStatus
    {
        PARSING_INACTIVE,
        PARSING_IN_PROGRESS,
        PARSING_SUCCESS,
        PARSING_FAILURE
    };

    typedef std::map<std::string, std::string> RxMessageArg;
    RxMessageArg data;
    MessageParser *child;
    ParsingStatus parsingStatus;

    void parseMessageBuffer(std::string *inBuff);
    void parseKeyValuePair(std::string *);
    // bool parseKeyValuePair(std::string *, std::string *, std::string *);
    void printMessage();
    bool isNode()
    {
        return child == nullptr;
    }
    void printParsingStatusInfo()
    {
#if OUTPUT_DEBUG_INFO
        switch(this->parsingStatus)
        {
            case ParsingStatus::PARSING_INACTIVE:
                std::cout << "PARSING_INACTIVE" << std::endl;
                break;
            case ParsingStatus::PARSING_IN_PROGRESS:
                std::cout << "PARSING_IN_PROGRESS" << std::endl;
                break;
            case ParsingStatus::PARSING_SUCCESS:
                std::cout << "PARSING_SUCCESS" << std::endl;
                break;
            case ParsingStatus::PARSING_FAILURE:
                std::cout << "PARSING_FAILURE" << std::endl;
                break;
            default:
                std::cout << "Something weird." << std::endl;
                break;
        }
#endif
    }
    bool succeeded()
    {
        std::cout << "Final: ";
        printParsingStatusInfo();
        return (this->parsingStatus == ParsingStatus::PARSING_SUCCESS);
    }

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
