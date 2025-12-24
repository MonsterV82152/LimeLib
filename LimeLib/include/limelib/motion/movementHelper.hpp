#pragma once

#ifndef LIMELIB_MOTION_MOVEMENTHELPER_HPP
#define LIMELIB_MOTION_MOVEMENTHELPER_HPP

#include "limelib/types.hpp"
#include "limelib/geometry/pose2d.hpp"
#include "limelib/geometry/point2d.hpp"
#include "pros/rtos.hpp"
#include <atomic>

namespace limelib
{
    class Timer
    {
    public:
        Timer(int milis);
        void update();
        int elapsedTime();
        bool isDone();
        void restart(int milis);

    private:
        int duration;
        int elapsed;
        int startTime;
    };

    class MovementHelper
    {
    public:
        MovementHelper();
        bool isDone();
        int elapsedTime();
        void cancel();
        void reset(int milis);
        void waitUntilDone();
        static real_t getAngleDiff(real_t targetAngle, real_t currentAngle);
        static real_t getDistance(const Pose2D &targetPose, const Pose2D &currentPose);
        static real_t getDistance(const Point2D &targetPose, const Pose2D &currentPose);
        static real_t getDistance(const Pose2D &targetPose, const Point2D &currentPose);
        static real_t getDistance(const Point2D &targetPose, const Point2D &currentPose);
        static std::pair<real_t, real_t> desaturate(real_t throttle, real_t turn, real_t maxMagnitude = 127);


    private:
        std::atomic<bool> cancelled;
        Timer timer;
    };
}

#endif