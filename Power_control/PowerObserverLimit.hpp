#ifndef POWER_OBSERVER_LIMIT_HPP
#define POWER_OBSERVER_LIMIT_HPP

#include "PowerObserverLimit.h"
#include <cmath>

namespace power {

class ObserverGate {
public:
    ObserverGate()
    {
        PowerObsCtrlParam param;
        PowerObsCtrl_DefaultParam(&param);
        PowerObsCtrl_Init(&ctrl_, &param);
    }

    explicit ObserverGate(const PowerObsCtrlParam &param)
    {
        PowerObsCtrl_Init(&ctrl_, &param);
    }

    void reset(const PowerObsCtrlParam &param)
    {
        PowerObsCtrl_Init(&ctrl_, &param);
    }

    float computeLambda(float power_limit, float power_buffer,
                        float predicted_power, float dt_s)
    {
        return PowerObsCtrl_ComputeLambda(&ctrl_, power_limit,
                                          power_buffer, predicted_power, dt_s);
    }

    PowerObsOutput apply(const PowerObsInput &in) const
    {
        PowerObsOutput out{};
        PowerObsCtrl_Apply(&ctrl_, &in, &out);
        return out;
    }

    float lambda() const { return ctrl_.lambda; }
    float predictedPower() const { return ctrl_.predicted_power; }
    float allowedPower() const { return ctrl_.allowed_power; }

    PowerObsCtrl       &raw()       { return ctrl_; }
    const PowerObsCtrl &raw() const { return ctrl_; }

private:
    PowerObsCtrl ctrl_;
};

} // namespace power

#endif
