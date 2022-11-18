#pragma once

#if defined(LFAST_TERMINAL)
#include "TerminalInterface.h"
#endif

class ServoInterface
{
protected:
#if defined(LFAST_TERMINAL)
    TerminalInterface *cli;
    bool terminalIsConnected{false};
#endif
public:
#if defined(LFAST_TERMINAL)
    virtual void connectTerminalInterface(TerminalInterface *_cli)
    {
        if (_cli != nullptr)
        {
            cli = _cli;
            terminalIsConnected = true;
        }
    }
#endif
    virtual void setDriverState(uint16_t) = 0;
    virtual void getDriverState() = 0;
    virtual void setControlMode(uint16_t) = 0;
    virtual void getControlMode() = 0;

    virtual void updatePositionCommand(double) = 0;
    virtual void updateVelocityCommand(double) = 0;
    virtual void updateTorqueCommand(double) = 0;

    virtual double getVelocityFeedback(bool updateConsole = false) = 0;
    virtual double getCurrentFeedback(bool updateConsole = false) = 0;
    virtual double getPositionFeedback(bool updateConsole = false) = 0;
#if defined(LFAST_TERMINAL)
    virtual void setupPersistentFields()
    {
        if (cli != nullptr)
        {
            cli->printPersistentFieldLabels();
        }
    }
    virtual void updateStatusField(unsigned fieldId, const std::string &val) = 0;
    virtual void readAndUpdateStatusField(unsigned fieldId) = 0;
    virtual void updateStatusFields() = 0;
#endif
};