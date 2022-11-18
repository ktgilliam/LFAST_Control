#pragma once

#include <cinttypes>
#include <string>
// #include <memory>

#include "ServoInterface.h"
#include "KinkoNamespace.h"
#include "modbus/modbus.h"

/* Temporary readability macro to avoid unused variables warnings */
#define KINCO_UNUSED(x) (void)x

namespace KINKO
{
    const int drive_i_peak = 36;
    const double amps2counts = (2048.0 / drive_i_peak) / 1.414;
    const double rpm2cps = 512 * 10000.0 / 1875;

    enum KincoPersistentFields
    {
        DRIVER_STATUS_ROW = 1,
        DRIVER_MODE_ROW,
        CMD_ROW,
        POSN_FB_ROW,
        VEL_FB_ROW,
        TRQ_FB_ROW
    };
}

class KinkoDriver : public ServoInterface
{
private:
    static uint16_t numDrivers;
    modbus_t *ctx;

protected:
    int16_t driverNodeId;

    template <typename T>
    T readDriverRegister(uint16_t modBusAddr);

    template <typename T>
    uint16_t writeDriverRegisters(uint16_t modBusAddr, T reg_value);

public:
    KinkoDriver(int16_t driverId);
    virtual ~KinkoDriver(){};

    void setDriverState(uint16_t) override;
    void getDriverState() override{};
    void setControlMode(uint16_t) override;
    void getControlMode() override{};
    void updatePositionCommand(double) override;
    void updateVelocityCommand(double) override;
    void updateTorqueCommand(double) override;

    double getVelocityFeedback(bool updateConsole = false) override;
    double getCurrentFeedback(bool updateConsole = false) override;
    double getPositionFeedback(bool updateConsole = false) override;

    void connectRTU(const char *device, int baud, char parity, int data_bit, int stop_bit);
#if defined(LFAST_TERMINAL)
    void connectTerminalInterface(TerminalInterface *_cli) override;
    void setupPersistentFields() override;
    void updateStatusField(unsigned fieldId, const std::string &val) override;
    void readAndUpdateStatusField(unsigned fieldId) override;
    void updateStatusFields() override;
// void test_show_drive_struc();
#endif
};