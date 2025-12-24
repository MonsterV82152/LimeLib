// using namespace pros;

#include "globals.hpp"
#include <iostream>

void on_center_button() {}

void initialize()
{

    pros::lcd::initialize();
    chassis.calibrate();
    master.rumble("..");
}

void disabled() {}
void competition_initialize() {}

void autonomous()
{
    chassis.setPose(-24, -24, 0);
    chassis.moveToPoint(-24, 24, 2000);
    chassis.turnToPoint(24, 24, 2000);
    chassis.moveToPoint(24, 24, 2000);
    chassis.turnToPoint(24, -24, 2000);
    chassis.moveToPoint(24, -24, 2000);
    chassis.turnToPoint(-24, -24, 2000);
    chassis.moveToPoint(-24, -24, 2000);
}

void opcontrol()
{
    chassis.setPose(-48, 24, 0);
    while (true)
    {
        pros::delay(20);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            chassis.setPose(0, 0, 0);
            chassis.turnToPoint(90, 2000);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        {
            angularPID.kP += 0.1f;
            std::cout << "Angular PID kP increased to: " << angularPID.kP << std::endl;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            angularPID.kP -= 0.1f;
            std::cout << "Angular PID kP decreased to: " << angularPID.kP << std::endl;
        }
        int throttle = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        leftDT.move(throttle + turn);
        rightDT.move(throttle - turn);
    }
}