/** \file lfast_mount_driver.h
    \brief LFAST Az/El Pedestal INDI Driver
    \author Kevin Gilliam
*/

#pragma once

#include <iostream>

class KarbonCANBus
{
    public:
        KarbonCANBus(int arg) : testVar(arg) {}
        void PrintHelloWorld();
        int getTestVar(){return testVar;}
    private:
        int testVar;
};
