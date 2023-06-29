/*
   INDI Developers Manual
   Tutorial #1

   "Hello INDI"

   We construct a most basic (and useless) device driver to illustate INDI.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/


#pragma once

#include "libindi/defaultdevice.h"

#define TEST1234 5
class LFAST_AHRS : public INDI::DefaultDevice
{
    public:
        LFAST_AHRS() = default;

    protected:
        bool Connect() override;
        bool Disconnect() override;
        const char *getDefaultName() override;
};
