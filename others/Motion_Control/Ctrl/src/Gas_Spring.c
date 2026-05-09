#include "Gas_Spring.h"

uint8_t gas_spring_enable = 0;

float Gas_Spring_GetForce(float L0_m)
{
    if (!gas_spring_enable)
        return 0.0f;

    float y = L0_m * 1000.0f;

    // if (y < 180.0f) y = 180.0f;
    // if (y > 370.0f) y = 370.0f;

    return 1.0 * (2.44e-6f * y * y * y - 2.80e-3f * y * y + 1.043f * y - 34.66f);
}
