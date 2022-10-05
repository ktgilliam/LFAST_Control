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

bool LFAST::RxMessage::parseMessage(std::string const &str)
{
    bool resFlag = true;
    static int tmp;

    std::regex r1(R"(^\{\"(\w+)\":(.+)\}$)");
    std::smatch m;

    if (std::regex_search(str, m, r1))
    {
        std::cout << "## " << tmp++ << "\tmatches:" << m.size() << std::endl;
        if (m.size() >= 2)
        {
            this->data[m[1]] = m[2];
            if (isObject(m[2]))
            {
                std::cout << "### IS OBJ:\t";
                this->child = new RxMessage(m[2]);
            }
            else
            {
                std::cout << "### IS ARG:\t";
                this->child = nullptr;
            }
            std::cout << m[1] << "\t" << m[2] << std::endl;
        }
        else
        {
            resFlag = false;
        }
    }
    return resFlag;
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