#include <string>
#include <vector>
#include <iomanip>
#include <limits>
#include <map>
#include <iostream>

#pragma once

#define MAX_DEPTH 3
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
    MessageParser() : parsingStatus(ParsingStatus::PARSING_INACTIVE), childNode(nullptr), parentNode(nullptr) {}
    MessageParser(std::string s)
        : MessageParser()
    {
        this->depth = 1;
        this->parseMessageBuffer(&s);
    }
    MessageParser(std::string s, MessageParser* parentPtr)
        : MessageParser()
    {
        this->parentNode = parentPtr;
        this->depth = parentNode->depth + 1;
        if(this->depth > MAX_DEPTH )
        {
            std::cout << __LINE__ << ": MAX DEPTH EXCEEDED(" << this->depth << ")\n";
            this->parsingStatus = ParsingStatus::MAX_DEPTH_EXCEEDED;
        }
        else
        {
            std::cout << __LINE__ << ": " << "Child Depth: " << this->depth << std::endl;
            this->parseMessageBuffer(&s);
        }
        print_map(this->data);
    }
    ~MessageParser();
    enum class ParsingStatus
    {
        PARSING_INACTIVE,
        PARSING_IN_PROGRESS,
        PARSING_SUCCESS,
        PARSING_FAILURE,
        MAX_DEPTH_EXCEEDED
    };

    std::map<std::string, std::string>  data;
    MessageParser *childNode;
    MessageParser *parentNode;
    ParsingStatus parsingStatus;

    void parseMessageBuffer(std::string *inBuff);
    void parseKeyValuePair(std::string *);
    void printParsingStatusInfo();
    void printMessage();

    template <typename T>
    inline T lookup(std::string const &);

    bool isNode()
    {
        return childNode == nullptr;
    }
    bool isRoot()
    {
        return parentNode == nullptr;
    }
    bool succeeded()
    {
        printParsingStatusInfo();
        return (this->parsingStatus == ParsingStatus::PARSING_SUCCESS);
    }

    // protected:
    unsigned int depth;

    std::string find(std::string const &);
};

// Generator template specializations
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

// Parser template specializations:

template <typename T>
inline T lookup(std::string const &)
{

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
