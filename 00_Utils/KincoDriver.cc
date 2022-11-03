#include "KincoDriver.h"

#include <modbus/modbus-rtu.h>
#include <cinttypes>

#include "KinkoNamespace.h"
#include "BitFieldUtil.h"

uint16_t KincoDriver::numDrivers = 0;

double convertSpeedIUtoRPM(int32_t speed_units);
double convertCurrIUtoAmp(int32_t current_units);
int32_t convertSpeedRPMtoIU(int16_t speed_rpm);

modbus_t *KincoDriver::mb = nullptr;
bool KincoDriver::modbusConnected = false;

KincoDriver::KincoDriver(int16_t driverId)
    : driverNodeId(driverId)

{
    numDrivers++;
}

template <typename T>
T KincoDriver::readDriverRegister(uint16_t modBusAddr)
{
    uint16_t result_code = 0;
    KINCO_UNUSED(result_code);
    ConversionBuffer32 rxBuff;
    // uint16_t numWords = sizeof(T) / 2;
    uint16_t numBytes = sizeof(T);
    if (!modbusConnected)
        return 0;

    // read *len Holding Register values from (slave) *driverNodeId, address *addr
    if (modbusClient->requestFrom(driverNodeId, HOLDING_REGISTERS, modBusAddr, numBytes))
    {
        for (int ii = 0; ii < numBytes; ii++)
        {
            if (modbusClient->available())
            {
                auto value = modbusClient->read();
                if (value >= 0)
                    rxBuff.PARTS.U8[ii] = (uint16_t)value;
                else
                    result_code = 2 + ii;
            }
        }
    }
    else
    {
        result_code = 1;
        connectionOkFlag = false;
        // TODO send error to terminal interface
        return static_cast<T>(0);
    }
    if (numBytes == 2)
        return static_cast<T>(rxBuff.PARTS.UNSIGNED.LOWER);
    else
        return static_cast<T>(rxBuff.U32);
}

template <typename T>
uint16_t KincoDriver::writeDriverRegister(uint16_t modBusAddr, T reg_value)
{

    if (!modbusConnected)
        return 0;
    uint16_t result_code = 0;
    uint16_t numWords = sizeof(T) / 2;
    //  uint16_t numBytes = sizeof(T);
    ConversionBuffer32 txBuff; //, rxBuff;
    txBuff.U32 = reg_value;

    // write len register values to (slave) id driverNodeId, starting at address addr
    modbusClient->beginTransmission(driverNodeId, HOLDING_REGISTERS, modBusAddr, numWords);
    for (int ii = 0; ii < numWords; ii++)
    {
        modbusClient->write(txBuff.PARTS.U16[ii]);
    }

    // for (int ii = 0; ii < numWords; ii++)
    // {
    //     if (modbusClient->available())
    //     {
    //         auto value = modbusClient->read();
    //         if (value >= 0)
    //             rxBuff.PARTS.U16[ii] = (uint16_t)value;
    //         else
    //             result_code = 2 + ii;
    //     }
    // }
    if (!modbusClient->endTransmission())
    {
        // TODO: Send these message to the terminal
        //  Serial.print("failed writing to modbus! ");
        //  Serial.println(ModbusRTUClient.lastError());
        connectionOkFlag = false;
        result_code = 1;
    }
    else
    {
        connectionOkFlag = true;
        // if(print_debug_lib) Serial.println("success");
        result_code = 0;
    }
    delayMicroseconds(2000);
    return result_code;
}

void KincoDriver::setDriverState(uint16_t motor_state)
{
    writeDriverRegister<uint16_t>(KINKO::CONTROL_WORD, motor_state);
}

void KincoDriver::setControlMode(uint16_t motor_mode)
{
    writeDriverRegister<uint16_t>(KINKO::OPERATION_MODE, motor_mode);
}

void KincoDriver::updateVelocityCommand(double velocity_setpoint)
{
    // int32_t target_speed_value = convertSpeedRPMtoIU(velocity_setpoint);
    // int32_t target_speed_value = 27300;
    int32_t target_speed_value = 0x00085555;
    writeDriverRegister<int32_t>(KINKO::TARGET_SPEED, target_speed_value);
}

void KincoDriver::updateTorqueCommand(double torque_setpoint)
{
    writeDriverRegister<int32_t>(KINKO::TARGET_TORQUE, torque_setpoint);
}

double KincoDriver::getVelocityFeedback()
{
    // int32_t real_speed_units = readDriverRegister(KINKO::REAL_SPEED);
    auto real_speed_units = readDriverRegister<int32_t>(KINKO::REAL_SPEED);
    auto real_speed_rpm = convertSpeedIUtoRPM(real_speed_units);
    return real_speed_rpm;
}

double KincoDriver::getCurrentFeedback()
{
    auto real_current_units = readDriverRegister<int32_t>(KINKO::REAL_CURRENT);
    int32_t real_current_amps = convertCurrIUtoAmp(real_current_units);
    return real_current_amps;
}

double KincoDriver::getPositionFeedback()
{
    auto encoder_counts = readDriverRegister<int32_t>(KINKO::POS_ACTUAL);
    return encoder_counts;
}

double convertSpeedIUtoRPM(int32_t speed_units)
{
    return (((double)speed_units * 1875) / (512 * 10000));
}

double convertCurrIUtoAmp(int32_t current_units)
{
    return (double)current_units / KINKO::amps2counts * 100.0;
}

int32_t convertSpeedRPMtoIU(int16_t speed_rpm)
{
    int32_t speed_units = 0;
    double holder = 0;
    holder = (double)speed_rpm * KINKO::rpm2cps;
    speed_units = (int32_t)holder;
    return speed_units;
}

void KincoDriver::initModbusInterface(const char *device, int baud, char parity,
                                        int data_bit, int stop_bit)
{
    bool modbusSetupResult = true;
    mb = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);
    
    modbusSetupResult &= modbus_connect(mb) == 0;
    modbusSetupResult &= modbus_rtu_set_serial_mode(mb, MODBUS_RTU_RS485) == 0;


    modbusConnected = modbusSetupResult;
}