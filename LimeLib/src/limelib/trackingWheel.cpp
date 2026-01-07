#include "limelib/trackingWheel.hpp"

limelib::TrackingWheel::TrackingWheel(pros::Rotation *sensor, real_t diameter, real_t offset)
    : sensor(sensor), diameter(diameter), offset(offset), lastPos(0), isDifferential(false), left(nullptr), right(nullptr)
{
}

limelib::TrackingWheel::TrackingWheel(pros::MotorGroup *left, pros::MotorGroup *right, real_t diameter, real_t gearRatio)
    : left(left), right(right), diameter(diameter), offset(0), lastPos(0), isDifferential(true), sensor(nullptr), gearRatio(gearRatio)
{
}

limelib::real_t limelib::TrackingWheel::getDistanceTravelled()
{
    real_t currentPos;
    if (isDifferential)
    {
        real_t leftPose = 0.0;
        real_t rightPose = 0.0;
        for (int i = 0; i < left->size(); i++)
        {
            leftPose += left->get_position(i);
        }
        for (int i = 0; i < right->size(); i++)
        {
            rightPose += right->get_position(i);
        }  
        currentPos = (leftPose / left->size() + rightPose / right->size()) / 2 * gearRatio;
    }
    else
    {
        currentPos = sensor->get_position() / 100;
    }

    real_t distance = (currentPos - lastPos) * (diameter * M_PI / 360.0);
    lastPos = currentPos;
    return distance;
}

limelib::real_t limelib::TrackingWheel::getDiameter() const
{
    return diameter;
}

limelib::real_t limelib::TrackingWheel::getOffset() const
{
    return offset;
}

void limelib::TrackingWheel::calibrate()
{
    if (isDifferential)
    {
        for (int i = 0; i < left->size(); i++)
        {
            left->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES, i);
            left->tare_position(i);
        }
        for (int i = 0; i < right->size(); i++)
        {
            right->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES, i);
            right->tare_position(i);
        }
    }
    else
    {
        sensor->reset_position();
        sensor->reset();
    }
}