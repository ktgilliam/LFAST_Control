#pragma once

#include <cinttypes>
#include <string>
#include <vector>
// #include <memory>

#include "ServoInterface.h"
#include "KincoNamespace.h"
#include "modbus/modbus.h"

/* Temporary readability macro to avoid unused variables warnings */
#define KINCO_UNUSED(x) (void)x

namespace KINCO
{

    constexpr int drive_i_peak = 36;
    constexpr double amps2counts = (2048.0 / drive_i_peak) / 1.414;
    constexpr double counts2amps = 1.0 / amps2counts;
    constexpr double cps2rpm = 1875.0 / (512 * 10000);
    constexpr double rpm2cps = 512.0 * 10000.0 / 1875;
    constexpr int32_t deg2counts = (int32_t)COUNTS_PER_REV / 360;
    constexpr double counts2deg = 360.0 / COUNTS_PER_REV;

    enum KincoPersistentFields
    {
        DRIVER_STATUS_ROW = 1,
        DRIVER_MODE_ROW,
        CMD_ROW_A,
        CMD_ROW_B,
        EMPTY_ROW_0,
        POSN_FB_ROW_A,
        POSN_FB_ROW_B,
        EMPTY_ROW_1,
        VEL_FB_ROW_A,
        VEL_FB_ROW_B,
        EMPTY_ROW_2,
        TRQ_FB_ROW_A,
        TRQ_FB_ROW_B,
        NUM_ROWS
    };
}

class KincoDriver : public ServoInterface
{
private:
    static modbus_t *ctx;
    bool modbusNodeIsSet;
    bool DriveIsConnected;

protected:
    int16_t driverNodeId;


    static std::vector<KincoDriver *> connectedDrives;

    int32_t encoderOffset;
    // std::vector<uint16_t>
public:
    template <typename T>
    static T readDriverRegister(uint8_t devId, uint16_t modBusAddr);

    template <typename T>
    static uint16_t writeDriverRegisters(uint8_t devId, uint16_t modBusAddr, T reg_value);
    KincoDriver(int16_t driverId);
    virtual ~KincoDriver(){};
    static bool readyForModbus();
    void setDriverState(uint16_t) override;
    void getDriverState() override{};
    void setControlMode(uint16_t) override;
    void getControlMode() override{};
    void setDirectionMode(uint8_t dir);
    void setMaxSpeed(double maxRPM);
    void zeroPositionOffset();

    void updatePositionCommand(double) override;
    void updateVelocityCommand(double) override;
    void updateTorqueCommand(double) override;
    
    void updateVelocityLimit(double velocity_limit);

    double getVelocityFeedback(bool updateConsole = false) override;
    double getCurrentFeedback(bool updateConsole = false) override;
    double getPositionFeedback(bool updateConsole = false) override;

    static void initializeRTU(const char *device, int baud = 19200, char parity = 'N', int data_bit = 8, int stop_bit = 1);
    static bool rtuIsActive();
    bool driverHandshake();
#if defined(LFAST_TERMINAL)
    void connectTerminalInterface(TerminalInterface *_cli) override;
    void setupPersistentFields() override;
    void updateStatusField(unsigned fieldId, const std::string &val) override;
    void readAndUpdateStatusField(unsigned fieldId) override;
    void updateStatusFields() override;
// void test_show_drive_struc();
#endif
};