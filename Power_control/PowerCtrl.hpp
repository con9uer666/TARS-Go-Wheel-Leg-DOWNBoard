#ifndef POWERCTRL_HPP
#define POWERCTRL_HPP

#include "PowerCtrl.h"
#include "PowerObserverLimit.hpp"
#include "RLS.hpp"

namespace power {

class Controller {
public:
    static void init()
    {
        PowerCtralInit(&whell_power);
    }

    static void update()
    {
        PowerCtrl();
    }

    static void setEnable(bool enable)
    {
        PowerCtrl_SetEnable(enable ? 1 : 0);
    }

    static void setObserverGateEnable(bool enable)
    {
        PowerCtrl_SetObserverGateEnable(enable ? 1 : 0);
    }

    static void setBufferPidEnable(bool enable)
    {
        PowerCtrl_SetBufferPidEnable(enable ? 1 : 0);
    }

    static void setBufferTarget(float target)
    {
        PowerCtrl_SetBufferTarget(target);
    }

    static void applyObserverGate(float &body_distance_error,
                                  float &speed_error,
                                  float &yaw_error,
                                  float &d_yaw)
    {
        PowerCtrl_ApplyObserverGate(&body_distance_error, &speed_error,
                                    &yaw_error, &d_yaw);
    }

    static bool isEnabled()        { return g_power_ctrl_enable != 0; }
    static bool isGateEnabled()    { return g_power_obs_gate_enable != 0; }
    static bool isBufferPidEnabled() { return g_power_buffer_pid_enable != 0; }

    static float lambda()          { return g_power_obs_lambda; }
    static float bufferTarget()    { return g_power_buffer_target; }
    static float bufferMeasure()   { return g_power_buffer_measure; }
    static float bufferPidOut()    { return g_power_buffer_pid_out; }

    static ChassisPower       &state()       { return whell_power; }
    static const ChassisPower &cstate()      { return whell_power; }
};

} // namespace power

#endif
