#include "limelib/motion/pid.hpp"
#include <cmath>

limelib::PID::PID(real_t kP, real_t kI, real_t kD, real_t windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

limelib::real_t limelib::PID::update(const real_t error) {
    // calculate integral
    integral += error;

    if (sgn(error) != sgn(prevError) && signFlipReset)
        integral = 0;

    if (fabs(error) > windupRange && windupRange != 0)
        integral = 0;

    // calculate derivative
    const real_t derivative = error - prevError;
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
}

void limelib::PID::reset() {
    integral = 0;
    prevError = 0;
}
