#include "limelib/trackingWheel.hpp"


limelib::TrackingWheel::TrackingWheel(pros::Rotation* sensor, real_t diameter, real_t offset)
    : sensor(sensor), diameter(diameter), offset(offset), lastPos(0), isDifferential(false), left(nullptr), right(nullptr) {
}

limelib::TrackingWheel::TrackingWheel(pros::MotorGroup* left, pros::MotorGroup* right, real_t diameter)
    : left(left), right(right), diameter(diameter), offset(0), lastPos(0), isDifferential(true), sensor(nullptr) {
}

limelib::real_t limelib::TrackingWheel::getDistanceTravelled() {
    real_t currentPos;
    if (isDifferential) {
        real_t leftPose = 0.0;
        real_t rightPose = 0.0;
        for (int i; i < left->size(); i++) {
            leftPose += left->get_position(i);
        }
        for (int i; i < right->size(); i++) {
            rightPose += right->get_position(i);
        }
        currentPos = (leftPose / left->size() + rightPose / right->size()) / 2;
    } else {
        currentPos = sensor->get_position()/100;
    }
    
    real_t distance = (currentPos - lastPos) * (diameter * M_PI / 360.0);
    lastPos = currentPos;
    return distance;
}

limelib::real_t limelib::TrackingWheel::getDiameter() const {
    return diameter;
}

limelib::real_t limelib::TrackingWheel::getOffset() const {
    return offset;
}


void limelib::TrackingWheel::calibrate() {
    if (isDifferential) {
        right->tare_position();
        left->tare_position();
    } else {
        sensor->reset_position();
        sensor->reset();
    }
}