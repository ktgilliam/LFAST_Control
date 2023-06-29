/*
   INDI Developers Manual
   Tutorial #1

   "Hello INDI"

   We construct a most basic (and useless) device driver to illustrate INDI.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file LFAST_AHRS.cpp
    \brief Brief description
    \author Kevin Gilliam

    \example LFAST_AHRS.cpp
    More description
**/

#include "LFAST_AHRS.h"

#include <cstdio>
#include <memory>

const uint64_t tmp = TEST1234;

// std::unique_ptr<LFAST_AHRS> LFAST_AHRS(new LFAST_AHRS());
// LFAST_AHRS
/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool LFAST_AHRS::Connect()
{
    IDMessage(getDeviceName(), "Simple device connected successfully!");
    return true;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool LFAST_AHRS::Disconnect()
{
    IDMessage(getDeviceName(), "Simple device disconnected successfully!");
    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *LFAST_AHRS::getDefaultName()
{
    return "Simple Device";
}
