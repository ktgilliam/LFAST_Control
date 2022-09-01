/** \file lfast_mount_driver.h
    \brief LFAST Az/El Pedestal INDI Driver
    \author Kevin Gilliam
*/

#pragma once

#include "inditelescope.h"

class LFASTMount : public INDI::Telescope
{
    public:
        LFASTMount();

    protected:
        bool Handshake() override;

        const char *getDefaultName() override;
        bool initProperties() override;

        // Telescope specific functions
        bool ReadScopeStatus() override;
        bool Goto(double, double) override;
        bool Abort() override;

    private:
        double currentRA {0};
        double currentDEC {90};
        double targetRA {0};
        double targetDEC {0};

        // Debug channel to write mount logs to
        // Default INDI::Logger debugging/logging channel are Message, Warn, Error, and Debug
        // Since scope information can be _very_ verbose, we create another channel SCOPE specifically
        // for extra debug logs. This way the user can turn it on/off as desired.
        uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };

        // slew rate, degrees/s
        static const uint8_t SLEW_RATE = 3;
};
