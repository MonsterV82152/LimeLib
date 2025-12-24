#pragma once

#ifndef LIMELIB_CHASSIS_HPP
#define LIMELIB_CHASSIS_HPP

#include "limelib/types.hpp"
#include "limelib/locator.hpp"
#include "limelib/motion/pid.hpp"
#include "limelib/motion/trapezoidalMotionProfiler.hpp"
#include "limelib/motion/movementHelper.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <memory>

namespace limelib
{
    /**
     * Parameters for moveToPoint function
     */
    struct moveToPointParams
    {
        bool forwards = true;
        real_t maxSpeed = 127;
        real_t minSpeed = 0;
        real_t earlyExitRange = 0.0;
    };
    /**
     * Parameters for moveToPose function
     */
    struct moveToPoseParams
    {
        bool lateralForwards = true;
        bool angularForwards = true;
        real_t maxSpeed = 127;
        real_t minSpeed = 0;
        real_t earlyExitRange = 0.0;
    };
    /**
     * Parameters for turnToHeading function
     */
    struct turnToHeadingParams
    {
        bool forwards = true;
        real_t maxSpeed = 127;
        real_t minSpeed = 0;
        real_t earlyExitRange = 0.0;
    };
    /**
     * Class representing a robot chassis
     */
    class Chassis
    {
    public:
        /**
         * Constructor for Chassis
         * @param locator Reference to a Locator object for localization
         * @param leftDr Reference to the left motor group
         * @param rightDr Reference to the right motor group
         * @param lateralController Reference to the lateral PID controller
         * @param angularController Reference to the angular PID controller
         */
        Chassis(Locator &locator, pros::MotorGroup &leftDr, pros::MotorGroup &rightDr, PID &lateralController, PID &angularController);
        void calibrate();
        void cancelAllMovement();
        void setPose(Pose2D pose);
        void setPose(real_t x, real_t y, real_t theta);
        void setPID(PID &linearController, PID &angularController);
        void moveToPoint(Point2D point, int timeout, moveToPointParams params = moveToPointParams());
        void moveToPoint(real_t x, real_t y, int timeout, moveToPointParams params = moveToPointParams());
        void moveToPose(real_t x, real_t y, real_t theta, int timeout, moveToPoseParams params = moveToPoseParams());
        void turnToHeading(real_t heading, int timeout, turnToHeadingParams params = turnToHeadingParams());
        void turnToPoint(Point2D point, int timeout, turnToHeadingParams params = turnToHeadingParams());
        void turnToPoint(real_t x, real_t y, int timeout, turnToHeadingParams params = turnToHeadingParams());

    private:
        Locator &locator;
        MovementHelper movementHelper;
        pros::MotorGroup &leftDr;
        pros::MotorGroup &rightDr;
        PID &linearController;
        PID &angularController;
        bool isMoving = false;
        bool motionQueued = false;
        std::unique_ptr<pros::Task> movementTask;

        void moveToPointTask(Point2D point, int timeout, moveToPointParams params);
        void turnToHeadingTask(real_t heading, int timeout, turnToHeadingParams params = turnToHeadingParams());
        void turnToPointTask(Point2D point, int timeout, turnToHeadingParams params = turnToHeadingParams());
    };
}

#endif