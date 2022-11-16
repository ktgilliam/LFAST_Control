#pragma once


#include <cinttypes>
#include <limits>

namespace DIGITAL_CONTROL
{
    constexpr double pos_inf = std::numeric_limits<double>::infinity();
    constexpr double neg_inf = -1 * pos_inf;

    const uint8_t P_BIT = 0b001;
    const uint8_t I_BIT = 0b010;
    const uint8_t D_BIT = 0b100;

    enum
    {
        P = P_BIT,
        I = I_BIT,
        PI = P_BIT | I_BIT,
        D = D_BIT,
        PD = P_BIT | D_BIT,
        ID = I_BIT | D_BIT,
        PID = P_BIT | I_BIT | D_BIT
    };
};

class PID_Controller
{
public:
    PID_Controller(double _kp = 0.0, double _ki = 0.0, double _kd = 0.0);
    virtual ~PID_Controller() {}
    void configureGains(double _kp, double _ki, double _kd);
    void configureIntegratorSaturation(double ulim, double llim);
    void configureOutputSaturation(double ulim, double llim);
    void resetIntegrator();
    void reset();
    void update(double e, double dt, double *uC);
    bool integratorIsSaturated();
    bool outputIsSaturated();

private:
    struct limits
    {
        double ulim;
        double llim;
    };

    double Kp;
    double Ki;
    double Kd;

    double integratorState;
    double e_prev;
    bool firstTime;
    bool outputSaturatedFlag;

    struct limits integrator_limits
    {
        DIGITAL_CONTROL::pos_inf, DIGITAL_CONTROL::neg_inf
    };
    struct limits output_limits
    {
        DIGITAL_CONTROL::pos_inf, DIGITAL_CONTROL::neg_inf
    };

    bool limit_integrator;
    bool limit_output;
    uint8_t compensationMode;
    void configureCompMode();
    // bool anti_windup;
};