#include "limelib/motion/chassis.hpp"
#include <optional>
#include <algorithm>
#include <cmath>

limelib::Chassis::Chassis(Locator &locator, pros::MotorGroup &leftDr, pros::MotorGroup &rightDr, PID &lateralController, PID &angularController)
    : locator(locator), leftDr(leftDr), rightDr(rightDr), linearController(lateralController), angularController(angularController), movementTask(nullptr), movementHelper()
{
}

void limelib::Chassis::calibrate()
{
    locator.calibrate();
    angularController.reset();
    linearController.reset();
    movementHelper.cancel();
    movementTask = std::make_unique<pros::Task>([]() {}, "LIMELIB_CHASSIS_IDLE_TASK");
}

void limelib::Chassis::cancelAllMovement()
{
    movementHelper.cancel();
}

void limelib::Chassis::waitUntilDone()
{
    movementHelper.waitUntilDone();
}

void limelib::Chassis::setPose(Pose2D pose)
{
    locator.setPose(pose);
}

void limelib::Chassis::setPose(real_t x, real_t y, real_t theta, bool radians)
{
    locator.setPose(Pose2D{x, y, theta}, radians);
}
void limelib::Chassis::setPID(PID &lateralController, PID &angularController)
{
    this->linearController.kP = lateralController.kP;
    this->linearController.kI = lateralController.kI;
    this->linearController.kD = lateralController.kD;
    this->angularController.kP = angularController.kP;
    this->angularController.kI = angularController.kI;
    this->angularController.kD = angularController.kD;
}

void limelib::Chassis::moveToPoint(Point2D point, int timeout, moveToPointParams params)
{
    movementHelper.waitUntilDone();
    pros::delay(20); // Small delay to ensure previous movement has fully stopped
    std::cout << "Starting moveToPoint to (" << point.x << ", " << point.y << ") with timeout " << timeout << " ms\n";
    movementHelper.reset(timeout);
    movementTask->create([this, point, timeout, params]()
                         { moveToPointTask(point, timeout, params); }, "LIMELIB_MOVE_TO_POINT");
}
void limelib::Chassis::moveToPointTask(Point2D point, int timeout, moveToPointParams params)
{
    // Initialize persistent variables
    Pose2D currentPose = locator.getPose();
    const real_t initialAngle = std::atan2(point.x - currentPose.x, point.y - currentPose.y) * 180 / M_PI;
    bool settling = false;
    std::optional<bool> prevSide = std::nullopt;
    real_t prevLateralOut = 0;
    real_t prevAngularOut = 0;
    int lastTime = pros::millis();

    while (!movementHelper.isDone())
    {
        currentPose = this->locator.getPose();
        real_t distance = MovementHelper::getDistance(point, currentPose);

        // Calculate delta time for slew rate limiting
        int currentTime = pros::millis();
        real_t deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
        lastTime = currentTime;

        // Check if robot is close enough to start settling (LemLib: 7.5 inches default)
        if (!settling && distance < params.settleDistance)
        {
            settling = true;
            // Cap max speed when settling to prevent overshoot
            params.maxSpeed = std::max(std::abs(prevLateralOut), static_cast<real_t>(60.0));
        }

        // Calculate target heading
        real_t targetHeading = params.forwards
                                   ? std::atan2(point.x - currentPose.x, point.y - currentPose.y) * 180 / M_PI
                                   : std::atan2(currentPose.x - point.x, currentPose.y - point.y) * 180 / M_PI;

        // Calculate angular error
        real_t angularError = MovementHelper::getAngleDiff(targetHeading, currentPose.theta);

        // This naturally prioritizes turning - lateral output scales down when heading is off
        real_t cosineScale = std::cos(angularError * M_PI / 180.0);

        // Calculate lateral error with cosine scaling
        real_t lateralError = distance * cosineScale;
        if (!params.forwards)
            lateralError = -lateralError;

        // Check exit conditions
        if (settling && std::abs(lateralError) <= params.earlyExitRange)
        {
            break;
        }

        // Side-based early exit for motion chaining (LemLib style)
        if (params.minSpeed != 0)
        {
            bool side = (currentPose.y - point.y) * -std::sin(initialAngle * M_PI / 180.0) <=
                        (currentPose.x - point.x) * std::cos(initialAngle * M_PI / 180.0) + params.earlyExitRange;
            if (!prevSide.has_value())
                prevSide = side;
            if (side != prevSide.value())
                break; // Crossed the target line, exit for motion chaining
            prevSide = side;
        }

        // Calculate lateral output
        real_t lateralOutput = linearController.update(lateralError);

        // Apply slew rate limiting (acceleration control)
        if (params.slew > 0 && !settling)
        {
            real_t maxChange = params.slew * deltaTime;
            real_t change = lateralOutput - prevLateralOut;
            if (std::abs(change) > maxChange)
            {
                lateralOutput = prevLateralOut + (change > 0 ? maxChange : -maxChange);
            }
        }

        // Constrain lateral speed
        lateralOutput = std::clamp(lateralOutput, -params.maxSpeed, params.maxSpeed);

        // Apply minimum speed for motion chaining (only when not settling)
        if (!settling && params.minSpeed != 0)
        {
            if (params.forwards && lateralOutput > 0 && lateralOutput < params.minSpeed)
            {
                lateralOutput = params.minSpeed;
            }
            else if (!params.forwards && lateralOutput < 0 && lateralOutput > -params.minSpeed)
            {
                lateralOutput = -params.minSpeed;
            }
        }

        // Calculate angular output - KEY: Disable when settling!
        real_t angularOutput = 0;
        if (!settling)
        {
            angularOutput = angularController.update(angularError);

            // Apply slew rate limiting
            if (params.slew > 0)
            {
                real_t maxChange = params.slew * deltaTime;
                real_t change = angularOutput - prevAngularOut;
                if (std::abs(change) > maxChange)
                {
                    angularOutput = prevAngularOut + (change > 0 ? maxChange : -maxChange);
                }
            }

            // Constrain angular speed
            angularOutput = std::clamp(angularOutput, -params.maxSpeed, params.maxSpeed);
        }

        // Drift compensation - limit speed based on turn radius to prevent slipping
        if (params.driftCompensation > 0 && std::abs(angularError) > 5)
        {
            // Approximate turn radius from angular error and distance
            real_t turnRadius = std::abs(distance / std::tan(angularError * M_PI / 180.0));
            if (turnRadius > 0.1)
            {                                                                                // Prevent division issues
                real_t maxSlipSpeed = std::sqrt(params.driftCompensation * turnRadius) * 20; // Scale to motor units
                lateralOutput = std::clamp(lateralOutput, -maxSlipSpeed, maxSlipSpeed);
            }
        }

        // If combined output would exceed max, reduce lateral to make room for angular
        real_t totalOutput = std::abs(angularOutput) + std::abs(lateralOutput);
        if (totalOutput > params.maxSpeed)
        {
            real_t overflow = totalOutput - params.maxSpeed;
            if (lateralOutput > 0)
                lateralOutput -= overflow;
            else
                lateralOutput += overflow;
        }

        // Store previous outputs for slew
        prevLateralOut = lateralOutput;
        prevAngularOut = angularOutput;

        // Desaturate and apply to motors
        std::pair<real_t, real_t> desaturated = MovementHelper::desaturate(lateralOutput, angularOutput, params.maxSpeed);

        leftDr.move(static_cast<int16_t>(desaturated.first));
        rightDr.move(static_cast<int16_t>(desaturated.second));

        pros::delay(10);
    }

    std::cout << "MoveToPoint complete. Final position: x" << currentPose.x << " y" << currentPose.y << " inches\n";

    // Only brake if not motion chaining
    if (params.minSpeed == 0)
    {
        leftDr.brake();
        rightDr.brake();
    }
}

void limelib::Chassis::moveToPoint(real_t x, real_t y, int timeout, moveToPointParams params)
{
    moveToPoint(Point2D{x, y}, timeout, params);
}

void limelib::Chassis::turnToHeading(real_t heading, int timeout, turnToHeadingParams params)
{
    movementHelper.waitUntilDone();
    pros::delay(20); // Small delay to ensure previous movement has fully stopped
    std::cout << "Starting turnToHeading to " << heading << " degrees with timeout " << timeout << " ms\n";
    movementHelper.reset(timeout);
    movementTask->create([this, heading, timeout, params]()
                         { turnToHeadingTask(heading, timeout, params); });
}

void limelib::Chassis::turnToHeadingTask(real_t heading, int timeout, turnToHeadingParams params)
{
    Pose2D currentPose = locator.getPose();
    real_t angleDiff;
    while (!this->movementHelper.isDone())
    {
        currentPose = this->locator.getPose();
        angleDiff = MovementHelper::getAngleDiff((params.forwards ? heading : heading + 180), currentPose.theta);
        if (std::abs(angleDiff) <= params.earlyExitRange)
        {
            movementHelper.cancel();
            break;
        }
        real_t angularOutput = angularController.update(angleDiff);
        if (std::abs(angularOutput) < params.minSpeed)
        {
            angularOutput = (angularOutput >= 0 ? 1 : -1) * params.minSpeed;
        }
        if (std::abs(angularOutput) > params.maxSpeed)
        {
            angularOutput = (angularOutput >= 0 ? 1 : -1) * params.maxSpeed;
        }
        leftDr.move(static_cast<int16_t>(angularOutput));
        rightDr.move(static_cast<int16_t>(-angularOutput));

        pros::delay(10);
    }
    std::cout << "TurnToHeading complete. Final error: " << angleDiff << " degrees\n";
    if (params.earlyExitRange == 0)
    {
        leftDr.brake();
        rightDr.brake();
    }
}

void limelib::Chassis::turnToPoint(Point2D point, int timeout, turnToHeadingParams params)
{
    movementHelper.waitUntilDone();
    pros::delay(20); // Small delay to ensure previous movement has fully stopped
    std::cout << "Starting turnToPoint to (" << point.x << ", " << point.y << ") with timeout " << timeout << " ms\n";
    movementHelper.reset(timeout);
    movementTask->create([this, point, timeout, params]()
                         { turnToPointTask(point, timeout, params); });
}

void limelib::Chassis::turnToPointTask(Point2D point, int timeout, turnToHeadingParams params)
{
    Pose2D currentPose = locator.getPose();
    real_t angleDiff;
    while (!this->movementHelper.isDone())
    {
        currentPose = this->locator.getPose();
        real_t targetHeading = params.forwards ? std::atan2(point.x - currentPose.x, point.y - currentPose.y) * 180 / M_PI : std::atan2(currentPose.x - point.x, currentPose.y - point.y) * 180 / M_PI;
        angleDiff = MovementHelper::getAngleDiff(targetHeading, currentPose.theta);
        if (std::abs(angleDiff) <= params.earlyExitRange)
        {
            break;
        }
        real_t angularOutput = angularController.update(angleDiff);
        if (std::abs(angularOutput) < params.minSpeed)
        {
            angularOutput = (angularOutput >= 0 ? 1 : -1) * params.minSpeed;
        }
        if (std::abs(angularOutput) > params.maxSpeed)
        {
            angularOutput = (angularOutput >= 0 ? 1 : -1) * params.maxSpeed;
        }
        leftDr.move(static_cast<int16_t>(angularOutput));
        rightDr.move(static_cast<int16_t>(-angularOutput));

        pros::delay(10);
    }
    std::cout << "TurnToPoint complete. Final error: " << angleDiff << " degrees\n";

    if (params.earlyExitRange == 0)
    {
        leftDr.brake();
        rightDr.brake();
    }
}

void limelib::Chassis::turnToPoint(real_t x, real_t y, int timeout, turnToHeadingParams params)
{
    turnToPoint(Point2D{x, y}, timeout, params);
}