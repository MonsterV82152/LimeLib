#pragma once
#ifndef LIMELIB_TRAPEZOIDALMOTIONPROFILER_HPP
#define LIMELIB_TRAPEZOIDALMOTIONPROFILER_HPP

#include "limelib/types.hpp"

namespace limelib
{

    class TrapezoidalMotionProfile
    {
    public:
        /**
         * Constructor for trapezoidal motion profile
         * @param maxSpeed Maximum speed in units/second (e.g., inches/second)
         * @param maxAccel Maximum acceleration in units/second²
         */
        TrapezoidalMotionProfile(real_t maxSpeed, real_t maxAccel);

        /**
         * Set maximum deceleration (allows asymmetric profiles)
         * @param maxDecel Maximum deceleration in units/second²
         */
        void setMaxDecel(real_t maxDecel);

        /**
         * Set maximum acceleration
         * @param maxAccel Maximum acceleration in units/second²
         */
        void setMaxAccel(real_t maxAccel);

        /**
         * Set maximum speed
         * @param maxSpeed Maximum speed in units/second
         */
        void setMaxSpeed(real_t maxSpeed);

        /**
         * Generate motion profile for given distance
         * @param distance Target distance to travel (can be negative for reverse)
         */
        void generateProfile(real_t distance);

        /**
         * Get position at given time
         * @param time Time since start of motion profile
         * @return Position at specified time
         */
        real_t getPosition(real_t time) const;

        /**
         * Get velocity at given time
         * @param time Time since start of motion profile
         * @return Velocity at specified time
         */
        real_t getVelocity(real_t time) const;

        /**
         * Get acceleration at given time
         * @param time Time since start of motion profile
         * @return Acceleration at specified time
         */
        real_t getAcceleration(real_t time) const;

        /**
         * Check if motion profile is complete at given time
         * @param time Time to check
         * @return True if motion is complete
         */
        bool isComplete(real_t time) const;

        /**
         * Get total time for the motion profile
         * @return Total time in seconds
         */
        real_t getTotalTime() const;

        /**
         * Get final position (should equal the distance requested)
         * @return Final position
         */
        real_t getFinalPosition() const;

        /**
         * Reset the profile (clears generated profile)
         */
        void reset();

        // Getters for profile parameters
        real_t getMaxSpeed() const { return maxSpeed; }
        real_t getMaxAccel() const { return maxAccel; }
        real_t getMaxDecel() const { return maxDecel; }

    private:
        real_t maxSpeed; // Maximum speed (units/second)
        real_t maxAccel; // Maximum acceleration (units/second²)
        real_t maxDecel; // Maximum deceleration (units/second²)

        // Profile parameters (calculated by generateProfile)
        real_t totalDistance; // Target distance
        real_t totalTime;     // Total time for motion
        real_t accelTime;     // Time spent accelerating
        real_t cruiseTime;    // Time spent at constant velocity
        real_t decelTime;     // Time spent decelerating

        bool profileGenerated; // Flag to track if profile has been generated
    };

} // namespace limelib

#endif