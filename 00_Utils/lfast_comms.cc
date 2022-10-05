#include "lfast_comms.h"

#include <string>
#include <sstream>
#include <cstring>
#include <vector>
#include <iomanip>
#include <iterator>

// template <>
// void LFAST::Message::addArgument(std::string label, const double &value)
// {
//     char pMSG[MAX_MSG_BUFF] = {0};
//     snprintf(pMSG, MAX_MSG_BUFF, "\"%s\":\"%g\"", label.c_str(), &value);
//     this->argStrings.push_back(std::string(pMSG));
// }

std::string LFAST::Message::getMessageStr()
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