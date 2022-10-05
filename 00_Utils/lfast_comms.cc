#include "lfast_comms.h"

#include <string>
#include <sstream>
#include <cstring>
#include <vector>
#include <iomanip>

template <typename T>
void LFAST::Message::addArgument(std::string label, T value)
{
    // char pMSG[MAX_MSG_BUFF] = {0};
    // snprintf(pMSG, MAX_MSG_BUFF, "\"%s\":\"%X\"", label.c_str(), value);
    // this->argStrings.push_back(std::string(pMSG));

    std::stringstream ss;
    ss << std::quoted(label) << ":" << std::hex << std::quoted(value);
    this->argStrings.push_back(ss.str());
}

template <>
void LFAST::Message::addArgument(std::string label, double value)
{
    char pMSG[MAX_MSG_BUFF] = {0};
    snprintf(pMSG, MAX_MSG_BUFF, "\"%s\":\"%g\"", label.c_str(), value);
    this->argStrings.push_back(std::string(pMSG));
}

template <>
void LFAST::Message::addArgument(std::string label, std::string value)
{
    std::stringstream ss;
    ss << std::quoted(label) << ":" << std::quoted(value);
    this->argStrings.push_back(ss.str());
}


template <>
void LFAST::Message::addArgument(std::string label, bool value)
{
    char pMSG[MAX_MSG_BUFF] = {0};
    if (value)
        snprintf(pMSG, MAX_MSG_BUFF, "\"%s\":True", label.c_str());
    else
        snprintf(pMSG, MAX_MSG_BUFF, "%s:False", label.c_str());
    this->argStrings.push_back(std::string(pMSG));
}

std::string LFAST::Message::getMessageStr()
{
    std::stringstream ss;
    ss << "{";
    if (this->DestIdStr != "")
    {
        ss << std::quoted(DestIdStr) << ":";
    }
    
    if(this->argStrings.size() > 0)
    {

    }
    else
    {
        ss << std::quoted("");
    }
    ss << "}";

    return ss.str();
}