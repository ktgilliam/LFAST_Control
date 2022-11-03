#pragma once

#include <cinttypes>

#include <modbus/modbus-rtu.h>

#include "KinkoNamespace.h"
#include "ServoInterface.h"

/* Temporary readability macro to avoid unused variables warnings */
#define KINCO_UNUSED(x) (void)x



namespace KINKO
{
    const int drive_i_peak = 36;
    const double amps2counts = (2048.0 / drive_i_peak) / 1.414;
    const double rpm2cps = 512 * 10000.0 / 1875;

}
class KincoDriver : ServoInterface
{
private:
    static uint16_t numDrivers;
    bool connectionOkFlag;
protected:
    int16_t driverNodeId;
    // RS485Class *rs485 = nullptr;
    // ModbusRTUClientClass *modbusClient = nullptr;
    static modbus_t *mb;
    static bool modbusConnected;
    template <typename T>
    T readDriverRegister(uint16_t modBusAddr);

    template <typename T>
    uint16_t writeDriverRegister(uint16_t modBusAddr, T reg_value);

public:
    KincoDriver(int16_t driverId);
    virtual ~KincoDriver(){};

    void setDriverState(uint16_t) override;
    void getDriverState() override{};
    void setControlMode(uint16_t) override;
    void getControlMode() override{};

    void updateVelocityCommand(double) override;
    void updateTorqueCommand(double) override;

    double getVelocityFeedback() override;
    double getCurrentFeedback() override;
    double getPositionFeedback() override;

    static void initModbusInterface(const char *device, int baud, char parity,
                                        int data_bit, int stop_bit);
    // void test_show_drive_struc();
};