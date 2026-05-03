#ifndef RLS_HPP
#define RLS_HPP

#include "RLS.h"

namespace power {

class RLSUpdater {
public:
    void init(ChassisPower *power)
    {
        PowerControl_AutoUpdateParamInit(power);
    }

    float update(float x1, float x2, float x3, float y, ChassisPower power)
    {
        return PowerControl_AutoUpdateParam(x1, x2, x3, y, power);
    }

    static float predictLoss(float x1, float x2, float x3,
                             const ChassisPower *power)
    {
        return PowerControl_ModelLossPredict(x1, x2, x3, power);
    }

    static float predictWheelPower(float wl, float wr, float il, float ir,
                                   const ChassisPower *power)
    {
        return PowerControl_WheelPowerPredict(wl, wr, il, ir, power);
    }

    static float singleWheelCurrentFromPower(float w, float power_limit,
                                             const ChassisPower *power)
    {
        return PowerControl_SingleWheelCurrentFromPower(w, power_limit, power);
    }
};

} // namespace power

#endif
