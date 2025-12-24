#include "limelib/motion/trapezoidalMotionProfiler.hpp"

limelib::TrapezoidalMotionProfile::TrapezoidalMotionProfile(real_t maxSpeed, real_t maxAccel)
    : maxSpeed(maxSpeed), maxAccel(maxAccel), maxDecel(maxAccel),
      totalDistance(0), totalTime(0), accelTime(0), cruiseTime(0),
      decelTime(0), profileGenerated(false) {}

void limelib::TrapezoidalMotionProfile::setMaxDecel(real_t maxDecel)
{
    this->maxDecel = maxDecel;
}

void limelib::TrapezoidalMotionProfile::setMaxAccel(real_t maxAccel)
{
    this->maxAccel = maxAccel;
}

void limelib::TrapezoidalMotionProfile::setMaxSpeed(real_t maxSpeed)
{
    this->maxSpeed = maxSpeed;
}

void limelib::TrapezoidalMotionProfile::generateProfile(real_t distance)
{
    totalDistance = distance;
    real_t absDistance = std::abs(distance);

    // Calculate time to reach max speed
    real_t accelTimeToMax = maxSpeed / maxAccel;
    real_t decelTimeToMax = maxSpeed / maxDecel;

    // Distance covered during acceleration and deceleration phases
    real_t accelDistance = 0.5 * maxAccel * accelTimeToMax * accelTimeToMax;
    real_t decelDistance = 0.5 * maxDecel * decelTimeToMax * decelTimeToMax;

    // Check if we can reach maximum speed
    if (accelDistance + decelDistance <= absDistance)
    {
        // Trapezoidal profile - can reach max speed
        accelTime = accelTimeToMax;
        decelTime = decelTimeToMax;

        // Calculate cruise phase
        real_t cruiseDistance = absDistance - accelDistance - decelDistance;
        cruiseTime = cruiseDistance / maxSpeed;
    }
    else
    {
        // Triangular profile - cannot reach max speed
        // Solve for peak velocity: v²/(2*a1) + v²/(2*a2) = distance
        real_t peakVelocity = std::sqrt(2 * absDistance * maxAccel * maxDecel / (maxAccel + maxDecel));

        accelTime = peakVelocity / maxAccel;
        decelTime = peakVelocity / maxDecel;
        cruiseTime = 0;
    }

    totalTime = accelTime + cruiseTime + decelTime;
    profileGenerated = true;
}

limelib::real_t limelib::TrapezoidalMotionProfile::getPosition(real_t time) const
{
    if (!profileGenerated)
        return 0.0;

    // Clamp time to valid range
    time = std::max(0.0, (double)std::min(time, totalTime));

    real_t position = 0.0;
    real_t direction = (totalDistance >= 0) ? 1.0 : -1.0;

    if (time <= accelTime)
    {
        // Acceleration phase: s = 0.5 * a * t²
        position = 0.5 * maxAccel * time * time;
    }
    else if (time <= accelTime + cruiseTime)
    {
        // Cruise phase: s = s_accel + v_max * (t - t_accel)
        real_t accelDistance = 0.5 * maxAccel * accelTime * accelTime;
        real_t cruiseVelocity = (cruiseTime > 0) ? maxSpeed : maxAccel * accelTime;
        position = accelDistance + cruiseVelocity * (time - accelTime);
    }
    else
    {
        // Deceleration phase: s = s_cruise + v_cruise * t_decel - 0.5 * a_decel * t_decel²
        real_t accelDistance = 0.5 * maxAccel * accelTime * accelTime;
        real_t cruiseVelocity = (cruiseTime > 0) ? maxSpeed : maxAccel * accelTime;
        real_t cruiseDistance = cruiseVelocity * cruiseTime;

        real_t decelElapsed = time - accelTime - cruiseTime;
        position = accelDistance + cruiseDistance +
                   cruiseVelocity * decelElapsed -
                   0.5 * maxDecel * decelElapsed * decelElapsed;
    }

    return position * direction;
}

limelib::real_t limelib::TrapezoidalMotionProfile::getVelocity(real_t time) const
{
    if (!profileGenerated)
        return 0.0;

    time = std::max(0.0, (double)std::min(time, totalTime));

    real_t velocity = 0.0;
    real_t direction = (totalDistance >= 0) ? 1.0 : -1.0;

    if (time <= accelTime)
    {
        // Acceleration phase: v = a * t
        velocity = maxAccel * time;
    }
    else if (time <= accelTime + cruiseTime)
    {
        // Cruise phase: v = v_max (or peak velocity for triangular)
        velocity = (cruiseTime > 0) ? maxSpeed : maxAccel * accelTime;
    }
    else if (time < totalTime)
    {
        // Deceleration phase: v = v_cruise - a_decel * (t - t_cruise_end)
        real_t cruiseVelocity = (cruiseTime > 0) ? maxSpeed : maxAccel * accelTime;
        real_t decelElapsed = time - accelTime - cruiseTime;
        velocity = cruiseVelocity - maxDecel * decelElapsed;
    }
    else
    {
        // Motion complete
        velocity = 0.0;
    }

    return velocity * direction;
}

limelib::real_t limelib::TrapezoidalMotionProfile::getAcceleration(real_t time) const
{
    if (!profileGenerated)
        return 0.0;

    time = std::max(0.0, (double)std::min(time, totalTime));

    real_t acceleration = 0.0;
    real_t direction = (totalDistance >= 0) ? 1.0 : -1.0;

    if (time < accelTime)
    {
        acceleration = maxAccel;
    }
    else if (time < accelTime + cruiseTime)
    {
        acceleration = 0.0; // Cruise phase
    }
    else if (time < totalTime)
    {
        acceleration = -maxDecel; // Deceleration phase
    }
    else
    {
        acceleration = 0.0; // Motion complete
    }

    return acceleration * direction;
}

bool limelib::TrapezoidalMotionProfile::isComplete(real_t time) const
{
    return time >= totalTime;
}

limelib::real_t limelib::TrapezoidalMotionProfile::getTotalTime() const
{
    return totalTime;
}

limelib::real_t limelib::TrapezoidalMotionProfile::getFinalPosition() const
{
    return totalDistance;
}

void limelib::TrapezoidalMotionProfile::reset()
{
    profileGenerated = false;
    totalTime = 0;
    accelTime = 0;
    cruiseTime = 0;
    decelTime = 0;
    totalDistance = 0;
}
