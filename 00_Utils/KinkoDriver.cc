#include "KinkoDriver.h"
#include <cstring>
#include <cinttypes>
#include <string>
#include <stdexcept>

#include "KinkoNamespace.h"
#include "BitFieldUtil.h"

#define ERR_BUFF_SIZE 60

modbus_t *KinkoDriver::ctx;
std::vector<KinkoDriver *> KinkoDriver::connectedDrives;

double convertPosnIUtoDeg(int32_t current_units);
double convertSpeedIUtoRPM(int32_t speed_units);
double convertCurrIUtoAmp(int32_t current_units);
int32_t convertSpeedRPMtoIU(int16_t speed_rpm);

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
KinkoDriver::KinkoDriver(int16_t driverId)
    : driverNodeId(driverId)
{
    ctx = NULL;
    modbusNodeIsSet = false;
    DriveIsConnected = false;
    encoderOffset = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::initializeRTU(const char *device, int baud, char parity, int data_bit, int stop_bit)
{
    ctx = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);

    if (ctx == NULL)
    {
        throw std::runtime_error("Unable to create the libmodbus context\n");
    }

    if (modbus_connect(ctx) == -1)
    {
        char errBuff[ERR_BUFF_SIZE];
        sprintf(errBuff, "Connection failed [%s]: %s\n", device, modbus_strerror(errno));
        modbus_free(ctx);
        throw std::runtime_error(errBuff);
    }
    modbus_flush(ctx);
}

bool KinkoDriver::rtuIsActive()
{
    return (ctx != NULL);
}

bool KinkoDriver::driverHandshake()
{
    if (!readyForModbus())
    {
        char errBuff[100];
        sprintf(errBuff, "driverHandshake ID %d::Modbus Not Ready.\n", driverNodeId);
        throw std::runtime_error(errBuff);
    }

    modbus_flush(ctx);
    KINKO::StatusWord_t driveStatus;
    driveStatus.ALL = readDriverRegister<uint16_t>(driverNodeId, KINKO::STATUS_WORD);
    bool commsFound = driveStatus.BITS.COMMUNICATION_FOUND;
    if (commsFound)
    {
        DriveIsConnected = true;
        connectedDrives.push_back(this);
    }
    else
    {
        char errBuff[ERR_BUFF_SIZE];
        sprintf(errBuff, "driverHandshake ID %d, Connection failed. Modbuss Error: %s\n", driverNodeId, modbus_strerror(errno));
        throw std::runtime_error(errBuff);
    }
    return commsFound;
}

bool KinkoDriver::readyForModbus()
{
    if (ctx == NULL)
    {
        return false;
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
T KinkoDriver::readDriverRegister(uint8_t devId, uint16_t modBusAddr)
{
    if (!readyForModbus())
    {
        char errBuff[100];
        sprintf(errBuff, "readDriverRegister [%d]::Drive not connected.\n", modBusAddr);
        throw std::runtime_error(errBuff);
    }

    modbus_set_slave(ctx, devId);
    uint16_t result_code = 0;
    constexpr uint16_t numWords = sizeof(T) / sizeof(uint16_t);
    ConversionBuffer<T> rxBuff;

    result_code = modbus_read_registers(ctx, modBusAddr, numWords, rxBuff.U16_PARTS);
    if (result_code == -1)
    {
        modbus_flush(ctx);
        throw std::runtime_error(modbus_strerror(errno));
    }
    return static_cast<T>(rxBuff.WHOLE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
uint16_t KinkoDriver::writeDriverRegisters(uint8_t devId, uint16_t modBusAddr, T reg_value)
{
    if (!readyForModbus())
    {
        char errBuff[100];
        sprintf(errBuff, "writeDriverRegisters [%d]::Drive not connected.\n", modBusAddr);
        throw std::runtime_error(errBuff);
    }

    modbus_set_slave(ctx, devId);
    uint16_t result_code = 0;
    uint16_t numWords = sizeof(T) / sizeof(uint16_t);

    ConversionBuffer<T> txBuff;
    if (numWords == 1)
    {
        result_code = modbus_write_register(ctx, modBusAddr, reg_value);
    }
    else
    {
        txBuff.WHOLE = reg_value;
        result_code = modbus_write_registers(ctx, modBusAddr, numWords, txBuff.U16_PARTS);
    }
    if (result_code == -1)
    {
        modbus_flush(ctx);
        throw std::runtime_error(modbus_strerror(errno));
    }
    return result_code;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::setDriverState(uint16_t motor_state)
{
    writeDriverRegisters<uint16_t>(driverNodeId, KINKO::CONTROL_WORD, motor_state);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::setControlMode(uint16_t motor_mode)
{
    writeDriverRegisters<uint16_t>(driverNodeId, KINKO::OPERATION_MODE, motor_mode);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        // enum motor_mode_enum modeSwitch = (enum motor_mode_enum) motor_mode;
        switch (motor_mode)
        {
        case KINKO::MOTOR_MODE_POSITION:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "POSITION MODE");
            break;
        case KINKO::MOTOR_MODE_SPEED:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "RATE MODE");
            break;
        case KINKO::MOTOR_MODE_TORQUE:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "TORQUE MODE");
            break;
        case KINKO::MOTOR_MODE_HOME:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "HOMING MODE");
            break;
        default:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "MODE UNKNOWN");
            break;
        }
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::setDirectionMode(uint8_t dir)
{
    writeDriverRegisters<int32_t>(driverNodeId, KINKO::INVERT_DIRECTION, dir);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::setMaxSpeed(double maxRPM)
{
    int32_t max_rpm_IU = convertSpeedRPMtoIU(maxRPM);
    writeDriverRegisters<int32_t>(driverNodeId, KINKO::MAX_SPEED, max_rpm_IU);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::zeroPositionOffset()
{

    encoderOffset = readDriverRegister<int32_t>(driverNodeId, KINKO::POS_ACTUAL);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::updatePositionCommand(double posn_setpoint)
{
    writeDriverRegisters<int32_t>(driverNodeId, KINKO::TARGET_POSITION, posn_setpoint);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        if (driverNodeId == 1)
            cli->updatePersistentField(KINKO::CMD_ROW_A, posn_setpoint);
        else if (driverNodeId == 2)
            cli->updatePersistentField(KINKO::CMD_ROW_B, posn_setpoint);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::updateVelocityCommand(double velocity_setpoint)
{
    int32_t target_speed_value = convertSpeedRPMtoIU(velocity_setpoint);
    writeDriverRegisters<int32_t>(driverNodeId, KINKO::TARGET_SPEED, target_speed_value);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        if (driverNodeId == 1)
            cli->updatePersistentField(KINKO::CMD_ROW_A, velocity_setpoint);
        else if (driverNodeId == 2)
            cli->updatePersistentField(KINKO::CMD_ROW_B, velocity_setpoint);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::updateTorqueCommand(double torque_setpoint)
{
    int16_t torque_sp_percent = (int16_t) (torque_setpoint * 100);
    writeDriverRegisters<int32_t>(driverNodeId, KINKO::TARGET_TORQUE, torque_sp_percent);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        if (driverNodeId == 1)
            cli->updatePersistentField(KINKO::CMD_ROW_A, torque_sp_percent);
        else if (driverNodeId == 2)
            cli->updatePersistentField(KINKO::CMD_ROW_B, torque_sp_percent);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KinkoDriver::getVelocityFeedback(bool updateConsole)
{
    // int32_t real_speed_units = readDriverRegister(KINKO::REAL_SPEED);
    auto real_speed_units = readDriverRegister<int32_t>(driverNodeId, KINKO::REAL_SPEED);
    auto real_speed_rpm = convertSpeedIUtoRPM(real_speed_units);
#if defined(LFAST_TERMINAL)
    if (updateConsole && cli != nullptr)
    {
        if (driverNodeId == 1)
            cli->updatePersistentField(KINKO::VEL_FB_ROW_A, real_speed_rpm);
        else if (driverNodeId == 2)
            cli->updatePersistentField(KINKO::VEL_FB_ROW_B, real_speed_rpm);
    }
#endif
    return real_speed_rpm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KinkoDriver::getCurrentFeedback(bool updateConsole)
{
    auto real_current_units = readDriverRegister<int16_t>(driverNodeId, KINKO::REAL_CURRENT);
    double real_current_amps = convertCurrIUtoAmp(real_current_units);

#if defined(LFAST_TERMINAL)
    if (updateConsole && cli != nullptr)
    {
        if (driverNodeId == 1)
            cli->updatePersistentField(KINKO::TRQ_FB_ROW_A, real_current_units);
        else if (driverNodeId == 2)
            cli->updatePersistentField(KINKO::TRQ_FB_ROW_B, real_current_units);
    }
#endif
    return real_current_amps;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KinkoDriver::getPositionFeedback(bool updateConsole)
{
    int32_t encoder_counts = readDriverRegister<int32_t>(driverNodeId, KINKO::POS_ACTUAL);
    int32_t encoder_counts_offs = encoder_counts - encoderOffset;
    // if(encoder_counts_offs < 0)
    //     encoder_counts_offs += KINKO::COUNTS_PER_REV;

    double encoder_pos = convertPosnIUtoDeg(encoder_counts_offs);

#if defined(LFAST_TERMINAL)
    if (updateConsole && cli != nullptr)
    {
        if (driverNodeId == 1)
            cli->updatePersistentField(KINKO::POSN_FB_ROW_A, encoder_counts_offs);
        else if (driverNodeId == 2)
            cli->updatePersistentField(KINKO::POSN_FB_ROW_B, encoder_counts_offs);
    }
#endif
    return encoder_pos;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertSpeedIUtoRPM(int32_t speed_units)
{
    return (((double)speed_units * 1875) / (512 * 10000));
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
int32_t convertSpeedRPMtoIU(int16_t speed_rpm)
{
    int32_t speed_units = 0;
    double holder = 0;
    holder = (double)speed_rpm * KINKO::rpm2cps;
    speed_units = (int32_t)holder;
    return speed_units;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertCurrIUtoAmp(int32_t current_units)
{
    return (double)current_units * KINKO::counts2amps;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertPosnIUtoDeg(int32_t current_units)
{
    return (double)current_units * KINKO::counts2deg;
}

#if defined(LFAST_TERMINAL)
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::connectTerminalInterface(TerminalInterface *_cli)
{
    ServoInterface::connectTerminalInterface(_cli);
    if (terminalIsConnected)
        setupPersistentFields();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::setupPersistentFields()
{

    // cli->addPersistentField("Driver Status", KINKO::DRIVER_STATUS_ROW);
    // cli->addPersistentField("Driver Mode", KINKO::DRIVER_MODE_ROW);
    cli->addPersistentField("A Command", KINKO::CMD_ROW_A);
    cli->addPersistentField("A Position FB [cnts]", KINKO::POSN_FB_ROW_A);
    cli->addPersistentField("A Velocity FB [RPM]", KINKO::VEL_FB_ROW_A);
    cli->addPersistentField("A Torque FB [%]", KINKO::TRQ_FB_ROW_A);

    cli->addPersistentField("B Command", KINKO::CMD_ROW_B);
    cli->addPersistentField("B Position FB [cnts]", KINKO::POSN_FB_ROW_B);
    cli->addPersistentField("B Velocity FB [RPM]", KINKO::VEL_FB_ROW_B);
    cli->addPersistentField("B Torque FB [%]", KINKO::TRQ_FB_ROW_B);

    ServoInterface::setupPersistentFields();
    updateStatusFields();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::updateStatusField(unsigned fieldId, const std::string &val)
{
    if (cli != nullptr)
    {
        cli->updatePersistentField(fieldId, val);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::readAndUpdateStatusField(unsigned fieldId)
{
    enum KINKO::KincoPersistentFields fieldSwitch = (enum KINKO::KincoPersistentFields)fieldId;

    switch (fieldSwitch)
    {
    case KINKO::DRIVER_MODE_ROW:
        // read driver mode
        // cli->updatePersistentField(fieldSwitch, "INIT");
        break;
    case KINKO::DRIVER_STATUS_ROW:
        // read driver status
        // cli->updatePersistentField(fieldSwitch, "INIT");
        break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KinkoDriver::updateStatusFields()
{
    cli->updatePersistentField(KINKO::DRIVER_MODE_ROW, "INIT");
    cli->updatePersistentField(KINKO::DRIVER_STATUS_ROW, "INIT");
}
#endif
