#include "limelib/motion/chassis.hpp"

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

void limelib::Chassis::setPose(Pose2D pose)
{
    locator.setPose(pose);
}

void limelib::Chassis::setPose(real_t x, real_t y, real_t theta)
{
    locator.setPose(Pose2D{x, y, theta});
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
    Pose2D currentPose = locator.getPose();
    real_t totalDistance = MovementHelper::getDistance(point, currentPose);
    bool approaching = false;
    while (!movementHelper.isDone())
    {
        currentPose = this->locator.getPose();
        real_t distance = MovementHelper::getDistance(point, currentPose);
        if (distance < 1)
        {
            approaching = true;
        }
        if (distance <= params.earlyExitRange)
        {
            break;
        }
        real_t forwardHeading = std::atan2(point.x - currentPose.x, point.y - currentPose.y) * 180 / M_PI;
        real_t backwardHeading = std::atan2(currentPose.x - point.x, currentPose.y - point.y) * 180 / M_PI;
        bool forwards = approaching ? (abs(forwardHeading - currentPose.theta) < abs(backwardHeading - currentPose.theta)) : params.forwards;
        real_t heading = params.forwards ? forwardHeading : backwardHeading;
        real_t angularError = approaching ? 0 : MovementHelper::getAngleDiff(heading, currentPose.theta);
        real_t angularOutput = angularController.update(angularError);
        distance = forwards ? distance : -distance;
        real_t lateralOutput = linearController.update(distance);
        std::pair<real_t, real_t> desaturated = MovementHelper::desaturate(lateralOutput, angularOutput, params.maxSpeed);
        leftDr.move(static_cast<int16_t>(desaturated.first));
        rightDr.move(static_cast<int16_t>(desaturated.second));

        pros::delay(10);
    }
    std::cout << "MoveToPoint complete. Final error: " << movementHelper.getDistance(point, locator.getPose()) << " inches\n";
    if (params.earlyExitRange == 0) {
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
    while (!this->movementHelper.isDone())
    {
        currentPose = this->locator.getPose();
        real_t angleDiff = MovementHelper::getAngleDiff(heading, currentPose.theta);
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
    std::cout << "TurnToHeading complete. Final error: " << movementHelper.getAngleDiff(heading, locator.getPose().theta) << " degrees\n";
    if (params.earlyExitRange == 0) {
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
    while (!this->movementHelper.isDone())
    {
        currentPose = this->locator.getPose();
        real_t targetHeading = std::atan2(point.x - currentPose.x, point.y - currentPose.y) * 180 / M_PI;
        real_t angleDiff = MovementHelper::getAngleDiff(targetHeading, currentPose.theta);
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
    std::cout << "TurnToPoint complete. Final error: " << movementHelper.getAngleDiff(std::atan2(point.x - locator.getPose().x, point.y - locator.getPose().y) * 180 / M_PI, locator.getPose().theta) << " degrees\n";
    
    if (params.earlyExitRange == 0) {
        leftDr.brake();
        rightDr.brake();
    }
}

void limelib::Chassis::turnToPoint(real_t x, real_t y, int timeout, turnToHeadingParams params)
{
    turnToPoint(Point2D{x, y}, timeout, params);
}