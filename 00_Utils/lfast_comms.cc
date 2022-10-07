#include "lfast_comms.h"

#include <string>
#include <sstream>
#include <cstring>
#include <vector>
#include <iomanip>
#include <iterator>
#include <regex>
#include <iostream>
#include <map>


std::string LFAST::MessageGenerator::getMessageStr()
{
    bool objFlag = false;
    std::stringstream ss;
    ss << "{";

    if (this->DestIdStr != "")
    {
        objFlag = true;
        ss << std::quoted(DestIdStr) << ":";
    }

    if (this->argStrings.size() > 0)
    {
        if (objFlag)
            ss << "{";
        auto itr = this->argStrings.begin();
        while (itr != this->argStrings.end())
        {
            ss << *itr;
            if (itr++ < this->argStrings.end() - 1)
            {
                ss << ",";
            }
        }
        if (objFlag)
            ss << "}";
    }
    else
    {
        ss << std::quoted("");
    }
    ss << "}";

    return ss.str();
}


bool LFAST::isObject(std::string str)
{
    std::regex rgx(R"(^\{.*\}$)");
    std::smatch m;
    return (std::regex_search(str, m, rgx));
}

bool LFAST::MessageParser::parseKeyValuePair(std::string *kvStr, std::string *keybuff, std::string *valbuff)
{
    std::pair<std::string, std::string> result;
    std::regex r(R"(^\"(\w+)\":(.+)$)");
    std::smatch m;
    if (std::regex_search(*kvStr, m, r))
    {
        while (!m.ready())
        {
        }
        if (m.size() == 3)
        {
            *keybuff = m[1].str();
            *valbuff = m[2].str();
            *kvStr = "";
            return true;
        }
    }
    return false;
}
bool LFAST::isKey(std::string const &str)
{
    auto c = str.back();
    if (c == ':')
        return true;
    else
        return false;
}

std::string LFAST::cleanupKey(std::string const &inStr)
{
    auto outStr = inStr;
    outStr.erase(std::remove(outStr.begin(), outStr.end(), '\"'), outStr.end());
    return outStr;
}

std::string extractLeadingValString(std::string *inBuff)
{
    std::smatch m;
    std::regex re;
    re = (R"((\w+),(.*))");
    if (std::regex_search(*inBuff, m, re))
    {
        while (!m.ready())
            ;
        auto retVal = m[1].str();
        *inBuff = m[2].str();

        std::cout << "\t Extracted: " << retVal << std::endl;
        std::cout << "\t Remaining: " << *inBuff << std::endl;
        return retVal;
    }
    else
    {
        auto inBuffCopy = *inBuff;
        *inBuff = "";
        return inBuffCopy;
    }
}

void LFAST::MessageParser::parseObject(std::string *inBuff)
{
    static int depth = 0;
    std::string keyStr = {0}, valStr = {0};
    std::smatch m;
    std::regex re;
    re = (R"(^\{\"(\w+)\":(.*)\}$)");
    if (std::regex_search(*inBuff, m, re))
    {

        while (!m.ready())
            ;
#if OUTPUT_DEBUG_INFO
        std::cout << "Parsing: ";
        for (unsigned ii = 1; ii < m.size(); ii++)
            std::cout << "<" << m[ii] << ">";
        std::cout << std::endl;
#endif
        auto preColonBuff = m[1].str();
        auto postColonBuff = m[2].str();
        keyStr = preColonBuff;

        if (isObject(postColonBuff))
        {
#if OUTPUT_DEBUG_INFO
            std::cout << "Found an object for parent " << keyStr << std::quoted(postColonBuff) << std::endl;
#endif
            valStr = postColonBuff;
            this->data[keyStr] = valStr;
            this->child = new MessageParser(postColonBuff);
        }
        else
        {

            if (postColonBuff.size() > 0)
            {
                do
                {
#if OUTPUT_DEBUG_INFO
                    std::cout << "Found a non object for parent " << keyStr << ": " << std::quoted(postColonBuff) << std::endl;
                    if (postColonBuff.size() <= 0)
                        std::cout << "Buffer empty" << std::endl;
#endif
                    std::string newValStr1 = extractLeadingValString(&postColonBuff);
                    this->data[keyStr] = newValStr1;
#if OUTPUT_DEBUG_INFO
                    std::cout << "Added <" << keyStr << ":" << newValStr1 << ">"
                              << "to key/value pair(s) [" << count++ << "]" << std::endl;
#endif
                    if (postColonBuff.size() > 0)
                    {
                        this->parseObject(&postColonBuff);
                    }
                    else
                    {
                        std::cout << "We're done here.\n";
                        this->child = nullptr;
                    }
                }
                while (postColonBuff.size() > 0);
            }
        }
    }
    else
    {
#if OUTPUT_DEBUG_INFO
        std::cout << "\t\t" << *inBuff << "...might be kv pairs?" << std::endl;
#endif
        std::string delimiter = ",";
        size_t pos = 0;
        std::string token;
        unsigned count = 0;
        while ((pos = inBuff->find(delimiter)) != std::string::npos)
        {
            token = inBuff->substr(0, pos);
            inBuff->erase(0, pos + delimiter.length());
#if OUTPUT_DEBUG_INFO
            std::cout << "\t\t\t TOKEN (" << ++count << ")" << token << std::endl;
#endif
            auto tmp = parseKeyValuePair(&token, &keyStr, &valStr);
#if OUTPUT_DEBUG_INFO
            std::cout << "\t\t\tParsed kv pair: " << keyStr << ":::::" << valStr << std::endl;
            std::cout << "\t\t\tRemaining in buffer: " << *inBuff << std::endl;
#endif
            this->data[keyStr] = valStr;
        }

        auto tmp = parseKeyValuePair(inBuff, &keyStr, &valStr);
#if OUTPUT_DEBUG_INFO
        std::cout << "\t\t\tParsed kv pair: " << keyStr << ":::::" << valStr << std::endl;
#endif
        this->data[keyStr] = valStr;
    }
}

void LFAST::MessageParser::printMessage()
{
    std::cout << std::endl;
}

void LFAST::print_map(std::map<std::string, std::string> const &m)
{
    std::for_each(m.begin(),
                  m.end(),
                  [](const std::pair<std::string, std::string> &p)
    {
        std::cout << "{" << p.first << ": " << p.second << "}\n";
    });
}