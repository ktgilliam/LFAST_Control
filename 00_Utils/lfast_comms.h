#include <string>
#include <vector>
#include <iomanip>
#include <limits>
#include <map>
#include <iostream>
#include <algorithm>

#pragma once

#define MAX_DEPTH 3
#define OUTPUT_DEBUG_INFO true

#if OUTPUT_DEBUG_INFO
#define DEBUG_PRINT_PARSE_STATUS() \
    std::cout << __LINE__ << ": "; \
    this->printParsingStatusInfo();
#else
#define DEBUG_PRINT_PARSE_STATUS() // no-op
#endif

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
        const char *getMessageCStr();

        std::string getArgString(int idx)
        {
            return argsList.at(idx);
        }
        std::string getArgKey(int idx)
        {
            // Remove all double-quote characters
            auto &argStr = argsList.at(idx);

            auto keyStr = argStr.substr(0, argStr.find(':'));
            keyStr.erase(
                std::remove(keyStr.begin(), keyStr.end(), '\"'),
                keyStr.end());
            return keyStr;
        }
        unsigned int numArgs() { return argsList.size(); }

    protected:
        std::vector<std::string> argsList;
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
        MessageParser(std::string s, MessageParser *parentPtr)
            : MessageParser()
        {
            this->parentNode = parentPtr;
            this->depth = parentNode->depth + 1;
            if (this->depth > MAX_DEPTH)
            {
#if OUTPUT_DEBUG_INFO
                std::cout << __LINE__ << ": MAX DEPTH EXCEEDED(" << this->depth << ")\n";
#endif
                this->parsingStatus = ParsingStatus::MAX_DEPTH_EXCEEDED;
            }
            else
            {
#if OUTPUT_DEBUG_INFO
                std::cout << __LINE__ << ": "
                          << "Child Depth: " << this->depth << std::endl;
#endif
                this->parseMessageBuffer(&s);
            }
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

        std::map<std::string, std::string> data;
        MessageParser *childNode;
        MessageParser *parentNode;
        ParsingStatus parsingStatus;

        void parseMessageBuffer(std::string *inBuff);
        void parseKeyValuePair(std::string *);
        void printParsingStatusInfo();
        std::string printMessage();

        template <typename T>
        inline bool lookup(std::string const &, T *);
        // inline T lookup(std::string const &);

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
            DEBUG_PRINT_PARSE_STATUS();
            return (this->parsingStatus == ParsingStatus::PARSING_SUCCESS);
        }

        protected:
        unsigned int depth;
        static bool isObject(std::string &str);
        bool find(std::string const &, std::string *);
        void cleanupString(std::string *);

    };

    // Generator template specializations
    template <typename T>
    inline void MessageGenerator::addArgument(std::string label, const T &value)
    {
        std::ostringstream ss;
        ss << std::quoted(label) << ":\"0x" << std::hex << value << "\"";
        this->argsList.push_back(ss.str());
    }
    template <std::size_t N>
    inline void MessageGenerator::addArgument(std::string label, const char (&value)[N])
    {
        std::stringstream ss;
        ss << std::quoted(label) << ":" << std::quoted(std::string(value));
        this->argsList.push_back(ss.str());
    }
    template <>
    inline void MessageGenerator::addArgument(std::string label, const unsigned int &value)
    {
        std::ostringstream ss;
        ss << std::quoted(label) << ":" << value;
        this->argsList.push_back(ss.str());
    }
    template <>
    inline void MessageGenerator::addArgument(std::string label, const int &value)
    {
        std::ostringstream ss;
        ss << std::quoted(label) << ":" << value;
        this->argsList.push_back(ss.str());
    }
    template <>
    inline void MessageGenerator::addArgument(std::string label, const bool &value)
    {
        std::ostringstream ss;
        ss << std::quoted(label) << ":" << std::boolalpha << value;
        this->argsList.push_back(ss.str());
    }

    template <>
    inline void MessageGenerator::addArgument(std::string label, const std::string &value)
    {
        std::stringstream ss;
        ss << std::quoted(label) << ":" << std::quoted(value);
        this->argsList.push_back(ss.str());
    }

    template <>
    inline void MessageGenerator::addArgument(std::string label, const double &value)
    {
        const int sigdigits = std::numeric_limits<float>::max_digits10;
        std::stringstream ss;
        ss << std::quoted(label) << ":" << std::setprecision(sigdigits) << value;
        this->argsList.push_back(ss.str());
    }

    inline void MessageGenerator::addArgument(std::string label, MessageGenerator &msg)
    {
        std::stringstream ss;

        ss << std::quoted(label) << ":" << msg.getMessageStr();

        this->argsList.push_back(ss.str());
    }

    inline void MessageGenerator::addArgument(std::string label)
    {
        std::stringstream ss;
        ss << std::quoted(label) << ":"
           << "null";
        this->argsList.push_back(ss.str());
    }

    // Parser template specializations:

    // template <typename T>
    // inline T MessageParser::lookup(std::string &keyStr)
    // {
    //     auto result = this->find(keyStr);
    //     return result;
    // }

    template <>
    inline bool MessageParser::lookup(std::string const &keyStr, std::string *resultStr)
    {
        bool resultFlag = this->find(keyStr, resultStr);
        resultStr->erase(
            std::remove(resultStr->begin(), resultStr->end(), '\"'),
            resultStr->end());
        return resultFlag;
    }

    template <>
    inline bool MessageParser::lookup(std::string const &keyStr, int *resultInt)
    {
        std::string resultStr = {0};
        bool resultFlag = this->find(keyStr, &resultStr);
        std::istringstream iss(resultStr);
        iss >> *resultInt;
        return resultFlag;
    }

    // Assumes base 10 unless 0x is prepended to value
    template <>
    inline bool MessageParser::lookup(std::string const &keyStr, unsigned int *resultUint)
    {
        std::string resultStr = {0};
        bool resultFlag = this->find(keyStr, &resultStr);
        // bool isHex = false;

        // if(resultStr.length() >= 3)
        // {
        //     auto checkStr = resultStr.substr(0, 2);
        //     if(checkStr.compare("0x") == 0)
        //     {
        //         isHex = true;
        //     }
        // }

        // if(isHex)
        // {
        //     resultInt = std::stoul(resultStr, nullptr, 16);
        // }
        // else
        // {
        // std::istringstream iss(resultStr);
        // iss >> resultInt;
        if (keyStr.length() > 0)
            *resultUint = std::stoul(resultStr, nullptr, 10);
        // }

        std::cout << "Result val: " << resultUint << std::endl;
        return resultFlag;
    }

    template <>
    inline bool MessageParser::lookup(std::string const &keyStr, bool *resultBool)
    {
        std::string resultStr = {0};
        bool resultFlag = this->find(keyStr, &resultStr);
        std::transform(resultStr.begin(), resultStr.end(), resultStr.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); });
        *resultBool = resultStr.compare("true") == 0;
        return resultFlag;
    }

    template <>
    inline bool MessageParser::lookup(std::string const &keyStr, double *resultDbl)
    {
        std::string resultStr = {0};
        bool resultFlag = this->find(keyStr, &resultStr);
        *resultDbl = std::atof(resultStr.c_str());
        return resultFlag;
    }

}
