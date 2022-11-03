#pragma once


class ServoInterface
{
protected:
    // TerminalInterface *cli = nullptr;

public:
    // void connectTerminalInterface(TerminalInterface *_cli);

    virtual void setDriverState(uint16_t) = 0;
    virtual void getDriverState() = 0;
    virtual void setControlMode(uint16_t) = 0;
    virtual void getControlMode() = 0;

    virtual void updateVelocityCommand(double) = 0;
    virtual void updateTorqueCommand(double) = 0;

    virtual double getVelocityFeedback() = 0;
    virtual double getCurrentFeedback() = 0;
    virtual double getPositionFeedback() = 0;
};