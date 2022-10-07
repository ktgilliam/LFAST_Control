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

// template <>
// void LFAST::Message::addArgument(std::string label, const double &value)
// {
//     char pMSG[MAX_MSG_BUFF] = {0};
//     snprintf(pMSG, MAX_MSG_BUFF, "\"%s\":\"%g\"", label.c_str(), &value);
//     this->argStrings.push_back(std::string(pMSG));
// }

std::string LFAST::TxMessage::getMessageStr()
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

bool LFAST::TxMessage::parseMessageStr(std::string const &str, unsigned int maxDepth)
{
    unsigned int nextMaxDepth = maxDepth - 1;
    if (nextMaxDepth > 0)
    {
        std::string gutsStr;
        // If it is an object, get the string of its guts
        // gutsStr = tryGetObjectContents(str);
    }

    return false;
}

bool LFAST::isObject(std::string str)
{
    std::regex rgx(R"(^\{.*\}$)");
    std::smatch m;
    return (std::regex_search(str, m, rgx));
}

bool LFAST::RxMessage::parseKeyValuePair(std::string *kvStr, std::string *keybuff, std::string *valbuff)
{
    std::pair<std::string, std::string> result;
    std::regex r(R"(^\"(\w+)\":(.+)$)");
    std::smatch m;
    // std::cout << "!!!" << *kvStr << "!!!" << std::endl;
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
    // outStr.erase(std::remove(outStr.begin(), outStr.end(), ':'), outStr.end());
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

void LFAST::RxMessage::parseObject(std::string *inBuff)
{
    static int count = 0;
    std::string keyStr = {0}, valStr = {0};
    std::smatch m;
    std::regex re;
    re = (R"(^\{\"(\w+)\":(.*)\}$)");
    if (std::regex_search(*inBuff, m, re))
    {

        while (!m.ready())
            ;
        std::cout << "Parsing: ";
        for (unsigned ii = 1; ii < m.size(); ii++)
            std::cout << "<" << m[ii] << ">";
        std::cout << std::endl;

        auto preColonBuff = m[1].str();
        auto postColonBuff = m[2].str();
        keyStr = preColonBuff;

        if (isObject(postColonBuff))
        {
            std::cout << "Found an object for parent " << keyStr << std::quoted(postColonBuff) << std::endl;
            valStr = postColonBuff;
            this->data[keyStr] = valStr;
            this->child = new RxMessage(postColonBuff);
        }
        else
        {

            if (postColonBuff.size() > 0)
            {
                do
                {
                    std::cout << "Found a non object for parent " << keyStr << ": " << std::quoted(postColonBuff) << std::endl;
                    std::string newValStr1 = extractLeadingValString(&postColonBuff);
                    if (postColonBuff.size() <= 0)
                        std::cout << "Buffer empty" << std::endl;
                    // keyStr = {0};/
                    this->data[keyStr] = newValStr1;
                    // valStr = {0};

                    std::cout << "Added <" << keyStr << ":" << newValStr1 << ">"
                              << "to key/value pair(s) [" << count++ << "]" << std::endl;
                    if (postColonBuff.size() > 0)
                    {
                        this->parseObject(&postColonBuff);
                        // std::cout << "\tParsing additional kv pair: " << std::endl;
                        // std::string newKeyStr = {0}, newValStr2 = {0};
                        // auto tmp = parseKeyValuePair(&postColonBuff, &newKeyStr, &newValStr2);
                        // this->data[keyStr] = valStr;
                        // std::cout << "Added <" << newKeyStr << ":" << newValStr2 << ">"
                        //           << "to key/value pair(s) [" << count++ << "]" << std::endl;
                    }
                    else
                    {
                        std::cout << "We're done here.\n";
                        this->child = nullptr;
                    }
                    // std::cout << keyStr << "::" << valStr << std::endl;
                } while (postColonBuff.size() > 0);
            }
        }
    }
    else
    {
        std::cout << "\t\t" << *inBuff << "...might be kv pairs?" << std::endl;

        bool multiFlag = false;
        std::string delimiter = ",";
        size_t pos = 0;
        std::string token;
        unsigned count = 0;
        while ((pos = inBuff->find(delimiter)) != std::string::npos)
        {
            multiFlag = true;
            token = inBuff->substr(0, pos);
            std::cout << "\t\t\t TOKEN (" << ++count << ")" << token << std::endl;
            inBuff->erase(0, pos + delimiter.length());
            auto tmp = parseKeyValuePair(&token, &keyStr, &valStr);
            std::cout << "\t\t\tParsed kv pair: " << keyStr << ":::::" << valStr << std::endl;
            this->data[keyStr] = valStr;
            std::cout << "\t\t\tRemaining in buffer: " << *inBuff << std::endl;
        }

        // if (!multiFlag)
        // {
            auto tmp = parseKeyValuePair(inBuff, &keyStr, &valStr);
            std::cout << "\t\t\tParsed kv pair: " << keyStr << ":::::" << valStr << std::endl;
            this->data[keyStr] = valStr;
        // }
        // auto tmpStr = extractLeadingValString(inBuff);
        // std::cout << "tmpStr::::: " << tmpStr << std::endl;
    }
}

std::string LFAST::RxMessage::parseMessage(std::string *buff)
{
    static int depth = 0;
    depth++;

    if (depth == 1)
        std::cout << "About to parse: " << *buff << std::endl
                  << std::endl;

    this->parseObject(buff);
    // std::cout << std::endl
    //           << "Printing Object " << depth << " MAP: ";
    // print_map(this->data);
    return "returnString";
    // }
}

void LFAST::RxMessage::printMessage()
{
    std::cout << std::endl;
}
// bool LFAST::tryGetObjectContents(std::string const &str, node )
// {
//     // std::regex rgx(R"(^\{(.+)\}$)");
//     bool isObject
//     std::regex rgx(R"(^\{\"(\w+)\":(.+)\}$)");
//     std::smatch m;
//     if (std::regex_search(str, m, rgx))
//     {
//         std::map<std::string, std::string> tmpKvMap{m[1], m[2]};
//         // tmpKvMap[m[1]] = m[2];
//         return true;
//     }
//     else
//     {
//         // Not an object
//     }
// }

// std::string LFAST::tryGetArrayContents(std::string const &str)
// {
//     std::regex rgx(R"(^\[(.+)\]$)");
//     std::smatch m;
//     if (std::regex_search(str, m, rgx))
//     {
//         return m[1].str();
//     }
//     else
//     {
//         return "";
//     }
// }

// bool LFAST::tryGetKeyValueMap(std::string const &str, std::map<std::string, std::string> &kvMap)
// {
//     std::regex rgx(R"(^\"(\w+)\":(.*))");
//     std::smatch m;
//     if (std::regex_search(str, m, rgx))
//     {
//         kvMap[m[1]] = m[2];
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

void LFAST::print_map(std::map<std::string, std::string> const &m)
{
    std::for_each(m.begin(),
                  m.end(),
                  [](const std::pair<std::string, std::string> &p)
                  {
                      std::cout << "{" << p.first << ": " << p.second << "}\n";
                  });
}