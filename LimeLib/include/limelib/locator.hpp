#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "limelib/types.hpp"
#include "limelib/geometry/pose2d.hpp"
#include "limelib/trackingWheel.hpp"
#include "limelib/geometry/field2d.hpp"
#include "limelib/geometry/ray2d.hpp"
#include "limelib/mclDistance.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/misc.hpp"
#include <memory>

namespace limelib
{
    struct MCLParticle
    {
        Pose2D point;
        real_t weight;

        MCLParticle(Pose2D point = Pose2D(), real_t weight = 0) : point(point), weight(weight) {}
    };
    real_t getRayCastDistance(const std::vector<LineSegment2D> &edges, Ray2D ray);

    // Velocity structure to hold linear and angular velocity
    struct Velocity
    {
        real_t linear;  // inches per second
        real_t angular; // radians per second
        real_t x;       // x velocity in inches per second
        real_t y;       // y velocity in inches per second

        Velocity(real_t linear = 0, real_t angular = 0, real_t x = 0, real_t y = 0)
            : linear(linear), angular(angular), x(x), y(y) {}
    };

    class Locator
    {
    public:
        // Add virtual destructor
        virtual ~Locator() = default;
        virtual Pose2D update();
        virtual void calibrate();
        virtual Pose2D getPose(bool radians = false) const = 0;
        virtual void setPose(Pose2D pose);
        virtual void setPose(real_t x, real_t y, real_t theta);
        virtual Velocity getVelocity() const = 0;
    };

    class Odometry : public Locator
    {
    public:
        /**
         * Odometry Constructor
         * @param verticalTW Pointer to the vertical tracking wheel
         * @param horizontalTW Pointer to the horizontal tracking wheel
         * @param imu Reference to the IMU sensor
         * @param shouldTaskRun Whether to run the odometry update in a separate task
         */
        Odometry(TrackingWheel *verticalTW, TrackingWheel *horizontalTW, pros::IMU &imu, bool shouldTaskRun = true);
        /**
         * Update the odometry and return the change in pose
         * @return Change in Pose2D since last update
         */
        Pose2D update() override;
        /**
         *  Calibrate the odometry system
         */
        void calibrate() override;
        /**
         * Get the current pose
         * @param radians Whether to return the heading in radians (default is degrees)
         */
        Pose2D getPose(bool radians = false) const override;
        /**
         * Set the current pose
         * @param pose Pose2D to set as the current pose
         */
        void setPose(Pose2D pose) override;
        /**
         * Set the current pose
         * @param x X coordinate
         * @param y Y coordinate
         * @param theta Heading angle
         */
        void setPose(real_t x, real_t y, real_t theta) override;
        /**
         * Get the current velocity
         * @return Current Velocity
         */
        Velocity getVelocity() const override;

    private:
        TrackingWheel *verticalTW;
        TrackingWheel *horizontalTW;
        pros::IMU &imu;
        Pose2D currentPose;
        Pose2D prevPose;
        real_t headingOffset;
        bool shouldTaskRun;
        std::unique_ptr<pros::Task> odomTask; // Task must persist!
        uint32_t lastUpdateTime;
        Velocity currentVelocity;
    };

    class MCL : public Locator
    {
    public:
        /**
         * MCL Localization Constructor
         * @param verticalTW Pointer to the vertical tracking wheel
         * @param horizontalTW Pointer to the horizontal tracking wheel
         * @param imu Reference to the IMU sensor
         * @param sensors Vector of MCLDistance sensors for distance measurements
         * @param field Reference to the Field2D object representing the environment
         * @param num_particles Number of particles to use in the MCL algorithm
         * @param rotationNoise Amount of rotation noise to add during prediction step
         * @param translationNoise Amount of translation noise to add during prediction step
         * @param debug Whether to enable debug display on the screen
         * @param intensitivity How often to update MCL (lower is more frequent)
         * @param shouldTaskRun Whether to run the MCL update in a separate task
         * @return MCL object
         *
         * @example
         * limelib::MCL mcl(&verticalTW, &horizontalTW, imu, sensors, field, 100, 0.1, 0.1, true, 10, true);
         *
         * This creates an MCL localization object using the specified tracking wheels, IMU, distance sensors, and field.
         * It uses 100 particles with 0.1 rotation and translation noise, enables debug display, updates every 10 cycles, and runs in a separate task.
         */
        MCL(TrackingWheel *verticalTW, TrackingWheel *horizontalTW, pros::Imu &imu, std::vector<MCLDistance> &sensors, Field2D &field, int num_particles, real_t rotationNoise, real_t translationNoise, bool debug = false, int intensitivity = 10, bool shouldTaskRun = true);
        /**
         * Calibrate the MCL localization system
         */
        void calibrate() override;
        /**
         * Update the MCL localization and return the estimated pose
         * @return Estimated Pose2D after MCL update
         */
        Pose2D update() override;
        /**
         * Get the current estimated pose
         * @param radians Whether to return the heading in radians (default is degrees)
         * @return Current estimated Pose2D
         */
        Pose2D getPose(bool radians = false) const override;
        /**
         * Set the current estimated pose
         * @param pose Pose2D to set as the current estimate
         */
        void setPose(Pose2D pose) override;
        /**
         * Set the current estimated pose
         * @param x X coordinate
         * @param y Y coordinate
         * @param theta Heading angle
         */
        void setPose(real_t x, real_t y, real_t theta) override;
        /**
         * Get the current estimated velocity
         * @return Current estimated Velocity
         */
        Velocity getVelocity() const override;

    private:
        Odometry odomHelper;
        pros::Controller master;
        std::vector<MCLDistance> &sensors;
        Field2D &field;
        Pose2D odomDelta;
        Pose2D poseOffset;
        Pose2D actualPose;
        MCLParticle estimatedPose;
        int NUM_PARTICLES;
        real_t ROTATION_NOISE;
        real_t TRANSLATION_NOISE;
        int INTENSITY;
        int last_mcl_update;
        int randomParticleCount;
        bool shouldTaskRun;
        bool debug;
        int debugCounter;
        std::vector<MCLParticle> particles;
        Pose2D prevPose;
        uint32_t lastUpdateTime;
        Velocity currentVelocity;
        std::unique_ptr<pros::Task> mclTask; // Task must persist!
        void updateMCL();
        void debugDisplay();
    };

}

#endif