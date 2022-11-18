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

    constexpr int drive_i_peak = 36;
    constexpr double amps2counts = (2048.0 / drive_i_peak) / 1.414;
    constexpr double counts2amps = 1.0 / amps2counts;
    constexpr double rpm2cps = 512 * 10000.0 / 1875;
    constexpr int32_t deg2counts = (int32_t)COUNTS_PER_REV / 360;
    constexpr double counts2deg = 360.0 / COUNTS_PER_REV;

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
    static modbus_t *ctx;
    bool modbusNodeIsSet;

protected:
    int16_t driverNodeId;

    template <typename T>
    T readDriverRegister(uint16_t modBusAddr);

    template <typename T>
    uint16_t writeDriverRegisters(uint16_t modBusAddr, T reg_value);

public:
    KinkoDriver(int16_t driverId);
    virtual ~KinkoDriver(){};
    bool readyForModbus();
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

    static void connectRTU(const char *device, int baud = 19200, char parity = 'N', int data_bit = 8, int stop_bit = 1);
    static bool IsConnected();
#if defined(LFAST_TERMINAL)
    void connectTerminalInterface(TerminalInterface *_cli) override;
    void setupPersistentFields() override;
    void updateStatusField(unsigned fieldId, const std::string &val) override;
    void readAndUpdateStatusField(unsigned fieldId) override;
    void updateStatusFields() override;
// void test_show_drive_struc();
#endif
};