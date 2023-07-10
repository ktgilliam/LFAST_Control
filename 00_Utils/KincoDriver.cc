#include "KincoDriver.h"
#include <cstring>
#include <cinttypes>
#include <string>
#include <stdexcept>

#include "KincoNamespace.h"
#include "BitFieldUtil.h"
#include "math_util.h"

#define ERR_BUFF_SIZE 60

modbus_t *KincoDriver::ctx;
std::vector<KincoDriver *> KincoDriver::connectedDrives;

double convertPosnIUtoDeg(int32_t current_units);
double convertSpeedIUtoRPM(int32_t speed_units);
double convertCurrIUtoAmp(int32_t current_units);
int32_t convertSpeedRPMtoIU(int16_t speed_rpm);

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
KincoDriver::KincoDriver(int16_t driverId)
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
void KincoDriver::initializeRTU(const char *device, int baud, char parity, int data_bit, int stop_bit)
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

bool KincoDriver::rtuIsActive()
{
    return (ctx != NULL);
}

bool KincoDriver::driverHandshake()
{
    if (!readyForModbus())
    {
        char errBuff[100];
        sprintf(errBuff, "driverHandshake ID %d::Modbus Not Ready.\n", driverNodeId);
        throw std::runtime_error(errBuff);
    }

    modbus_flush(ctx);
    readDriverStatus(false);
    checkFaults();
    bool commsFound = kincoStatusData.BITS.COMMUNICATION_FOUND;
    if (commsFound)
    {
        DriveIsConnected = true;
        connectedDrives.push_back(this);
    }
    else
    { 
        DriveIsConnected = false;
        char errBuff[ERR_BUFF_SIZE];
        sprintf(errBuff, "driverHandshake ID %d, Connection failed. Modbuss Error: %s\n", driverNodeId, modbus_strerror(errno));
        throw std::runtime_error(errBuff);
    }
    return commsFound;
}

void KincoDriver::checkFaults()
{
    if(kincoStatusData.BITS.FAULT == 1)
    {
        // writeDriverRegisters<uint16_t>(driverNodeId, KINCO::CONTROL_WORD, KINCO::POWER_OFF_MOTOR);
        // writeDriverRegisters<uint16_t>(driverNodeId, KINCO::CONTROL_WORD, KINCO::POWER_ON_MOTOR);
        // readDriverStatus(false);
        char errBuff[ERR_BUFF_SIZE];
        sprintf(errBuff, "driverHandshake ID %d, Driver Fault: %s\n", driverNodeId, modbus_strerror(errno));
        throw std::runtime_error(errBuff);
    }
}

bool KincoDriver::readyForModbus()
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
T KincoDriver::readDriverRegister(uint8_t devId, uint16_t modBusAddr)
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
uint16_t KincoDriver::writeDriverRegisters(uint8_t devId, uint16_t modBusAddr, T reg_value)
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
void KincoDriver::setDriverState(uint16_t motor_state)
{
    if (!DriveIsConnected)
        throw std::runtime_error("setDriverState: Driver connection not established (call driverHandshake() first).");
    writeDriverRegisters<uint16_t>(driverNodeId, KINCO::CONTROL_WORD, motor_state);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::setControlMode(uint16_t motor_mode)
{
    if (!DriveIsConnected)
        throw std::runtime_error("setControlMode: Driver connection not established (call driverHandshake() first).");
    writeDriverRegisters<uint16_t>(driverNodeId, KINCO::OPERATION_MODE, motor_mode);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        // enum motor_mode_enum modeSwitch = (enum motor_mode_enum) motor_mode;
        switch (motor_mode)
        {
        case KINCO::MOTOR_MODE_POSITION:
            updateStatusField(KINCO::DRIVER_MODE_ROW, "POSITION MODE");
            break;
        case KINCO::MOTOR_MODE_SPEED:
            updateStatusField(KINCO::DRIVER_MODE_ROW, "RATE MODE");
            break;
        case KINCO::MOTOR_MODE_TORQUE:
            updateStatusField(KINCO::DRIVER_MODE_ROW, "TORQUE MODE");
            break;
        case KINCO::MOTOR_MODE_HOME:
            updateStatusField(KINCO::DRIVER_MODE_ROW, "HOMING MODE");
            break;
        default:
            updateStatusField(KINCO::DRIVER_MODE_ROW, "MODE UNKNOWN");
            break;
        }
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::readDriverStatus(bool checkForHandshake)
{
    if (checkForHandshake)
    {
        if (!DriveIsConnected)
            throw std::runtime_error("readDriverStatus: Driver connection not established (call driverHandshake() first).");
    }
    kincoStatusData.ALL = readDriverRegister<int16_t>(driverNodeId, KINCO::STATUS_WORD);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::setDirectionMode(uint8_t dir)
{
    if (!DriveIsConnected)
        throw std::runtime_error("setDirectionMode: Driver connection not established (call driverHandshake() first).");
    writeDriverRegisters<int32_t>(driverNodeId, KINCO::INVERT_DIRECTION, dir);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::setMaxSpeed(double maxRPM)
{
    if (!DriveIsConnected)
        throw std::runtime_error("setMaxSpeed: Driver connection not established (call driverHandshake() first).");
    int32_t max_rpm_IU = (int32_t)maxRPM;
    writeDriverRegisters<int32_t>(driverNodeId, KINCO::MAX_SPEED, max_rpm_IU);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::zeroPositionOffset()
{
    if (!DriveIsConnected)
        throw std::runtime_error("zeroPositionOffset: Driver connection not established (call driverHandshake() first).");
    writeDriverRegisters<int32_t>(driverNodeId, KINCO::POS_ACTUAL, 0);
    encoderOffset = readDriverRegister<int32_t>(driverNodeId, KINCO::POS_ACTUAL);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updatePositionCommand(double posn_setpoint)
{
    if (!DriveIsConnected)
        throw std::runtime_error("updatePositionCommand: Driver connection not established (call driverHandshake() first).");
    writeDriverRegisters<int32_t>(driverNodeId, KINCO::TARGET_POSITION, posn_setpoint);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        if (driverNodeId == DRIVER_A_ID)
            cli->updatePersistentField(KINCO::CMD_ROW_A, posn_setpoint);
        else if (driverNodeId == DRIVER_B_ID)
            cli->updatePersistentField(KINCO::CMD_ROW_B, posn_setpoint);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updateVelocityCommand(double velocity_setpoint)
{
    if (!DriveIsConnected)
        throw std::runtime_error("updateVelocityCommand: Driver connection not established (call driverHandshake() first).");

    double vsp_saturated = saturate(velocity_setpoint, -KINCO::MOTOR_MAX_SPEED_RPM, KINCO::MOTOR_MAX_SPEED_RPM);
    int32_t vsp_IU = convertSpeedRPMtoIU(vsp_saturated);
    writeDriverRegisters<int32_t>(driverNodeId, KINCO::TARGET_SPEED, vsp_IU);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        if (driverNodeId == DRIVER_A_ID)
            cli->updatePersistentField(KINCO::CMD_ROW_A, velocity_setpoint);
        else if (driverNodeId == DRIVER_B_ID)
            cli->updatePersistentField(KINCO::CMD_ROW_B, velocity_setpoint);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updateTorqueCommand(double torque_setpoint)
{
    if (!DriveIsConnected)
        throw std::runtime_error("updateTorqueCommand: Driver connection not established (call driverHandshake() first).");
    int16_t torque_sp_percent = (int16_t)(torque_setpoint * 100 * 3.5);
    writeDriverRegisters<int32_t>(driverNodeId, KINCO::TARGET_TORQUE, torque_sp_percent);
#if defined(LFAST_TERMINAL)
    if (cli != nullptr)
    {
        if (driverNodeId == DRIVER_A_ID)
            cli->updatePersistentField(KINCO::CMD_ROW_A, torque_sp_percent);
        else if (driverNodeId == DRIVER_B_ID)
            cli->updatePersistentField(KINCO::CMD_ROW_B, torque_sp_percent);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updateVelocityLimit(double velocity_limit)
{
    if (!DriveIsConnected)
        throw std::runtime_error("updateVelocityLimit: Driver connection not established (call driverHandshake() first).");
    int32_t target_speed_value = convertSpeedRPMtoIU(velocity_limit);
    writeDriverRegisters<int32_t>(driverNodeId, KINCO::MAX_SPEED, target_speed_value);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KincoDriver::getVelocityFeedback(bool updateConsole)
{
    if (!DriveIsConnected)
        throw std::runtime_error("getVelocityFeedback: Driver connection not established (call driverHandshake() first).");
    // int32_t real_speed_units = readDriverRegister(KINCO::REAL_SPEED);
    auto real_speed_units = readDriverRegister<int32_t>(driverNodeId, KINCO::REAL_SPEED);
    auto real_speed_rpm = convertSpeedIUtoRPM(real_speed_units);
#if defined(LFAST_TERMINAL)
    if (updateConsole && cli != nullptr)
    {
        if (driverNodeId == DRIVER_A_ID)
            cli->updatePersistentField(KINCO::VEL_FB_ROW_A, real_speed_rpm);
        else if (driverNodeId == DRIVER_B_ID)
            cli->updatePersistentField(KINCO::VEL_FB_ROW_B, real_speed_rpm);
    }
#endif
    return real_speed_rpm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KincoDriver::getCurrentFeedback(bool updateConsole)
{
    if (!DriveIsConnected)
        throw std::runtime_error("getCurrentFeedback: Driver connection not established (call driverHandshake() first).");
    auto real_current_units = readDriverRegister<int16_t>(driverNodeId, KINCO::REAL_CURRENT);
    double real_current_amps = convertCurrIUtoAmp(real_current_units);

#if defined(LFAST_TERMINAL)
    if (updateConsole && cli != nullptr)
    {
        if (driverNodeId == DRIVER_A_ID)
            cli->updatePersistentField(KINCO::TRQ_FB_ROW_A, real_current_units);
        else if (driverNodeId == DRIVER_B_ID)
            cli->updatePersistentField(KINCO::TRQ_FB_ROW_B, real_current_units);
    }
#endif
    return real_current_amps;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KincoDriver::getPositionFeedback(bool updateConsole)
{
    if (!DriveIsConnected)
        throw std::runtime_error("getPositionFeedback: Driver connection not established (call driverHandshake() first).");
    int32_t encoder_counts = readDriverRegister<int32_t>(driverNodeId, KINCO::POS_ACTUAL);
    int32_t encoder_counts_offs = encoder_counts - encoderOffset;
    // if(encoder_counts_offs < 0)
    //     encoder_counts_offs += KINCO::COUNTS_PER_REV;

    double encoder_pos = convertPosnIUtoDeg(encoder_counts_offs);

#if defined(LFAST_TERMINAL)
    if (updateConsole && cli != nullptr)
    {
        if (driverNodeId == DRIVER_A_ID)
            cli->updatePersistentField(KINCO::POSN_FB_ROW_A, encoder_counts_offs);
        else if (driverNodeId == DRIVER_B_ID)
            cli->updatePersistentField(KINCO::POSN_FB_ROW_B, encoder_counts_offs);
    }
#endif
    return encoder_pos;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertSpeedIUtoRPM(int32_t speed_units)
{
    return (double)speed_units * KINCO::cps2rpm;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
int32_t convertSpeedRPMtoIU(int16_t speed_rpm)
{
    double holder = (double)speed_rpm * KINCO::rpm2cps;
    int32_t speed_units = (int32_t)holder;
    return speed_units;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertCurrIUtoAmp(int32_t current_units)
{
    return (double)current_units * KINCO::counts2amps;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertPosnIUtoDeg(int32_t current_units)
{
    return (double)current_units * KINCO::counts2deg;
}

#if defined(LFAST_TERMINAL)
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::connectTerminalInterface(TerminalInterface *_cli)
{
    ServoInterface::connectTerminalInterface(_cli);
    if (terminalIsConnected)
        setupPersistentFields();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::setupPersistentFields()
{

    // cli->addPersistentField("Driver Status", KINCO::DRIVER_STATUS_ROW);
    // cli->addPersistentField("Driver Mode", KINCO::DRIVER_MODE_ROW);
    cli->addPersistentField("A Command", KINCO::CMD_ROW_A);
    cli->addPersistentField("A Position FB [cnts]", KINCO::POSN_FB_ROW_A);
    cli->addPersistentField("A Velocity FB [RPM]", KINCO::VEL_FB_ROW_A);
    cli->addPersistentField("A Torque FB [%]", KINCO::TRQ_FB_ROW_A);

    cli->addPersistentField("B Command", KINCO::CMD_ROW_B);
    cli->addPersistentField("B Position FB [cnts]", KINCO::POSN_FB_ROW_B);
    cli->addPersistentField("B Velocity FB [RPM]", KINCO::VEL_FB_ROW_B);
    cli->addPersistentField("B Torque FB [%]", KINCO::TRQ_FB_ROW_B);

    ServoInterface::setupPersistentFields();
    updateStatusFields();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updateStatusField(unsigned fieldId, const std::string &val)
{
    if (cli != nullptr)
    {
        cli->updatePersistentField(fieldId, val);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::readAndUpdateStatusField(unsigned fieldId)
{
    enum KINCO::KincoPersistentFields fieldSwitch = (enum KINCO::KincoPersistentFields)fieldId;

    switch (fieldSwitch)
    {
    case KINCO::DRIVER_MODE_ROW:
        // read driver mode
        // cli->updatePersistentField(fieldSwitch, "INIT");
        break;
    case KINCO::DRIVER_STATUS_ROW:
        // read driver status
        // cli->updatePersistentField(fieldSwitch, "INIT");
        break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updateStatusFields()
{
    cli->updatePersistentField(KINCO::DRIVER_MODE_ROW, "INIT");
    cli->updatePersistentField(KINCO::DRIVER_STATUS_ROW, "INIT");
}
#endif
