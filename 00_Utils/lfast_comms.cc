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
// std::string splitMulti(std::string &inStr)
// {
//     std::regex r1(R"(^(\w+),(.*))");
//     std::smatch m1;
//     std::string outStr;
//     if (std::regex_search(inStr, m1, r1))
//     {
// #if 1
//         std::cout << "### ";
//         for (unsigned ii = 0; ii < m1.size(); ii++)
//             std::cout << "<<"
//                       << "(" << ii << ":" << m1[ii].str().size() << ")" << m1[ii] << ">>";
//         std::cout << std::endl;
// #endif
//         inStr = m1[2].str();
//         outStr = m1[1].str();
//     }
//     else
//     {
//         outStr = inStr;
//     }
//     std::cout << "*****" << m1[1].str() << "*****" << std::endl;
//     std::cout << "******" << m1[2].str() << "******" << std::endl;
//     return outStr;
// }
bool LFAST::RxMessage::parseKeyValuePair(std::string const &kvStr, std::string &keybuff, std::string &valbuff)
{
    std::pair<std::string, std::string> result;
    std::regex r(R"(^\"(\w+)\":(.+)$)");
    std::smatch m;
    std::cout << "!!!" << kvStr << "!!!" << std::endl;
    if (std::regex_search(kvStr, m, r))
    {
        while (!m.ready())
        {
        }
        if (m.size() == 3)
        {
            keybuff = m[1].str();
            valbuff = m[2].str();
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
int LFAST::RxMessage::parseMultipleArgs(std::string const &valStr)
{
    std::smatch m2;
    std::regex rComma(",");
    // std::vector<std::string> argStrs;

    std::regex_search(valStr, m2, rComma);
    while (!m2.ready())
    {
    }
    if ((m2.size() > 0))
    {

        std::cout << "###(" << m2.size() << ")";
        std::cout << "prefix: [" << m2.prefix() << "]\n";
        std::cout << "suffix: [" << m2.suffix() << "]\n";

        return m2.size();
    }
    else
    {
        return 1;
    }
}
// std::ostream &
std::string LFAST::RxMessage::parseMessage(std::string &buff, ParseStatus parentStatus)
{
    static int depth = 0;
    depth++;

    if (depth == 1)
        std::cout << buff << std::endl;

    std::smatch m;
    std::regex d;
    d = (R"([\{|,|:|\}])");

    // std::regex d(R"([,|:])");
    // std::vector<std::string> argStrs;
    std::string keyBuff = {0}, valBuff = {0};

    std::regex_search(buff, m, d);
    while (!m.ready())
    {
    }
    std::string returnString;
    if ((m.size() > 0))
    {
        auto delim = *(m[0].str().c_str());
        std::cout << "Delim(" << delim << ") ";

        auto firstPart = m.prefix().str();
        auto secondPart = m.suffix().str();

        // bool firstPartIsKey = false;
        bool secondPartIsValue = false;
        bool keepGoing = true;

        // std::string keyStr = {0}, valStr = {0};

        auto pad = std::string(depth, '\t');
        std::pair<std::string, std::string> kvPair;

        switch (parentStatus)
        {
        case ParseStatus::NEW_MESSAGE:
            std::cout << "# NewMsg:" << std::endl;
            std::cout << "(" << depth << ":" << m.size() << ")";
            std::cout << pad << "prefix: [" << m.prefix() << "]\n";
            std::cout << pad << "suffix: [" << m.suffix() << "]\n";
            parseMessage(secondPart, ParseStatus::NEW_OBJECT);
            // print_map(this->data);
            break;
        case ParseStatus::NEW_OBJECT:
            std::cout << "## NewObj:" << std::endl;
            std::cout << "(" << depth << ":" << m.size() << ")";
            std::cout << pad << "prefix: [" << m.prefix() << "]\n";
            std::cout << pad << "suffix: [" << m.suffix() << "]\n";

            // keyStr = firstPart;
            // valStr = parseMessage(secondPart, ParseStatus::PARENT_IS_KEY);
            // this->data[keyStr] = valStr;
            // if (isKey(firstPart))
            // {
            //     keyBuff = cleanupKey(firstPart);
            //     std::cout << "Keybuff: " << keyBuff << std::endl;
            // }
            if (parseKeyValuePair(firstPart, keyBuff, valBuff))
            {
                this->data[keyBuff] = valBuff;
                if (!isObject(valBuff))
                {
                    this->child = nullptr;
                }
            }
            else
            {
                std::cout << "fail" << std::endl;
            }
            break;
        case ParseStatus::PARENT_IS_KEY:
            std::cout << "### ParentIsKey:" << std::endl;
            std::cout << "(" << depth << ":" << m.size() << ")";
            std::cout << pad << "prefix: [" << m.prefix() << "]\n";
            std::cout << pad << "suffix: [" << m.suffix() << "]\n";
            if ((firstPart.size() > 0) && (secondPart.size() == 0))
            {
                returnString = firstPart;
                buff = "";
                this->child = nullptr;
            }
            else if ((firstPart.size() == 0) && (secondPart.size() > 0))
            {
                if (delim == '{')
                {
                    std::cout << "Child Object" << std::endl;
                    auto tmp = parseMessage(secondPart, ParseStatus::NEW_OBJECT);
                    std::cout << "childObj Returned: " << tmp << std::endl;
                }
                else if (delim == ',')
                {
                    std::cout << "Multi-Arg" << std::endl;
                    auto tmp = parseMessage(secondPart, ParseStatus::NEW_OBJECT);
                    std::cout << "multi-arg Returned: " << tmp << std::endl;
                }
                else
                {
                }
                // std::stringstream ss;
                // while (buff.size() > 0)
                // {
                //     std::cout << "(Inside a list)" << std::endl;
                //     if (depth < MAX_RX_DEPTH)
                //     {
                //         ss << parseMessage(buff, ParseStatus::PARENT_IS_KEY);
                //     }
                // }
                // returnString = ss.str();
            }

            break;
        default:
            // currentStatus = ParseStatus::SOMETHING_ELSE;
            break;
        }
        // if (firstPart.size() == 0)
        // {
        //     std::cout << "root.\n";
        // }
        // else
        // {

        // switch (*delim)
        // {
        // case '{':
        //     // std::cout << pad << "New object.\n";
        //     firstPartIsKey = true;
        //     break;

        // case ':':
        //     // std::cout << pad << "New value.\n";
        //     secondPartIsValue = true;
        //     break;
        //     // case ',':
        //     // case '}':
        // }
        // }
        // auto suf = m.suffix().str();
        // if (suf.size() > 0)
        // {
        //     auto notsureYet = parseMessage(suf, currentStatus);
        //     // std::cout << pad << "Result(" << suf.size() << "): " << retStr << std::endl;
        // }
        // else
        // {
        // }
        // return secondPartIsValue;
    }
    // else
    // {
    return returnString;
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