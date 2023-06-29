/*
   INDI Developers Manual
   Tutorial #1

   "Hello INDI"

   We construct a most basic (and useless) device driver to illustate INDI.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file LFAST_AHRS.h
    \brief Construct a basic INDI device with only one property to connect and disconnect.
    \author Jasem Mutlaq

    \example LFAST_AHRS.h
    A very minimal device! It also allows you to connect/disconnect and performs no other functions.
*/

#pragma once

#include "defaultdevice.h"

class LFAST_AHRS : public INDI::DefaultDevice
{
    public:
        LFAST_AHRS();

    protected:
        bool Connect() override;
        bool Disconnect() override;
        const char *getDefaultName() override;
};
