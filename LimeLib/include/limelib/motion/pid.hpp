#ifndef LIMELIB_PID_HPP
#define LIMELIB_PID_HPP

#include "limelib/types.hpp"

namespace limelib
{
    constexpr real_t sgn(real_t value) { return value < 0 ? -1 : 1; }
    class PID
    {
    public:
        PID(real_t kP, real_t kI, real_t kD, real_t windupRange = 0, bool signFlipReset = false);
        real_t update(real_t error);
        void reset();
        real_t kP;
        real_t kI;
        real_t kD;

    protected:
        const real_t windupRange;
        const bool signFlipReset;

        real_t integral = 0;
        real_t prevError = 0;
    };
}

#endif
