# LimeLib

**A robust Monte Carlo Localization (MCL) and motion control library for the VEX V5RC robotics platform.**

LimeLib is a high-performance C++ library built on [PROS](https://pros.cs.purdue.edu/) for VEX V5 competition robots. It provides accurate field-relative localization through both traditional odometry and particle-filter-based Monte Carlo Localization, paired with a full-featured chassis controller for autonomous navigation.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Documentation](#documentation)
  - [Coordinate System](#coordinate-system)
  - [Geometry Primitives](#geometry-primitives)
  - [Tracking Wheels](#tracking-wheels)
  - [Localization](#localization)
    - [Odometry](#odometry)
    - [Monte Carlo Localization (MCL)](#monte-carlo-localization-mcl)
  - [Motion Control](#motion-control)
    - [Chassis](#chassis)
    - [PID Controller](#pid-controller)
    - [Trapezoidal Motion Profiling](#trapezoidal-motion-profiling)
  - [Field Representation](#field-representation)
- [Configuration Reference](#configuration-reference)
- [Contributing](#contributing)

---

## Overview

Accurate localization is one of the hardest problems in competitive robotics. Wheel encoders drift over time, and even IMU-fused odometry accumulates error across a 15-second autonomous routine. LimeLib solves this by layering **Monte Carlo Localization** on top of wheel odometry — continuously correcting position estimates using onboard VEX Distance sensors and a raycasting model of the field environment.

Key design principles:

- **Memory efficiency** — uses `float` (`real_t`) throughout to minimize RAM usage on the V5 brain.
- **PROS task-based concurrency** — localization and motion run in background tasks so your user code stays clean.
- **Swappable locators** — `Odometry` and `MCL` share the same `Locator` interface; switching between them is a one-line change.
- **LemLib-compatible API** — motion functions and parameter structs will feel familiar to teams migrating from LemLib.

---

## Features

| Category | What's Included |
|---|---|
| **Localization** | Wheel odometry with IMU fusion, Monte Carlo Localization (particle filter) |
| **Sensors** | Rotation-sensor tracking wheels, motor-encoder tracking wheels, distance sensor wrappers |
| **Geometry** | `Point2D`, `Pose2D`, `LineSegment2D`, `Ray2D`, `Rectangle2D`, `Circle2D`, `Polygon2D`, `Line2D`, `Field2D` |
| **Motion Control** | `moveToPoint`, `moveToPose`, `turnToHeading`, `turnToPoint` with configurable parameters |
| **Controllers** | PID with integral windup protection and sign-flip reset; Trapezoidal motion profiler |
| **Utilities** | Angle delta, distance helpers, motor desaturation, timer |

---

## Requirements

- **VEX V5 Brain** running [PROS kernel 4.1.1+](https://pros.cs.purdue.edu/)
- **VEX IMU** (Inertial Sensor) — required for all localization modes
- **VEX Rotation Sensor(s)** or **Motor Groups** — for tracking wheel odometry
- **VEX Distance Sensors** — optional, required only for MCL
- C++17 or later (enabled by default in PROS)

---

## Quick Start

The example below sets up MCL localization with four distance sensors on a 144″ × 144″ VEX field, then drives a square in autonomous.

```cpp
// globals.hpp
#include "limelib/limelib.hpp"
using namespace limelib;

// --- Hardware ---
inline pros::MotorGroup leftDT({-1, -2, -3});
inline pros::MotorGroup rightDT({8, 9, 10});
inline pros::Rotation verticalSensor(15);
inline pros::Imu inertial(13);

// --- Distance sensors (for MCL) ---
inline pros::Distance rightDS(20), leftDS(5), frontDS(21), backDS(4);

// --- Tracking wheel: 2.75" diameter, -0.25" lateral offset ---
inline TrackingWheel verticalTW(&verticalSensor, 2.75, -0.25);

// --- Distance sensor wrappers with robot-relative poses ---
inline std::vector<MCLDistance> mclSensors = {
    {rightDS,  Pose2D( 4.25, -2.25,  90)},
    {leftDS,   Pose2D(-4.25, -2.25, 270)},
    {frontDS,  Pose2D( 4.5,   3.0,    0)},
    {backDS,   Pose2D(-3.5,  -5.5,  180)},
};

// --- Field (no obstacles for an open field) ---
inline Field2D field(144.0f, 144.0f);

// --- MCL locator ---
inline MCL locator(&verticalTW, nullptr, inertial, mclSensors, field,
                   /*particles=*/200, /*translationNoise=*/0.1f);

// --- PID controllers ---
inline PID linearPID(3.0, 0.0, 2.0);
inline PID angularPID(0.9, 0.0, 1.0);

// --- Chassis ---
inline Chassis chassis(locator, leftDT, rightDT, linearPID, angularPID);
```

```cpp
// main.cpp
#include "globals.hpp"

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // Calibrates IMU, tracking wheels, and starts localization task
}

void autonomous() {
    chassis.setPose(-24, -24, 0); // Starting position

    // Drive a 48" square
    chassis.moveToPoint(-24, 24, 2000);
    chassis.turnToPoint(24, 24, 2000);
    chassis.moveToPoint(24, 24, 2000);
    chassis.turnToPoint(24, -24, 2000);
    chassis.moveToPoint(24, -24, 2000);
    chassis.turnToPoint(-24, -24, 2000);
    chassis.moveToPoint(-24, -24, 2000);
}
```

---

## Documentation

### Coordinate System

LimeLib uses a **field-centric Cartesian coordinate system**:

- **X-axis** — positive points toward the right side of the field.
- **Y-axis** — positive points toward the top of the field.
- **Heading (θ)** — measured in degrees by default (0° = facing the positive Y direction / "up"). Can optionally be passed in radians by setting `radians = true`.
- All distances are in **inches**.
- The standard VEX V5RC field is **144″ × 144″**.

---

### Geometry Primitives

All geometry types live in the `limelib` namespace.

#### `Point2D`
A simple 2D point.
```cpp
limelib::Point2D p(12.0f, 24.0f); // x=12", y=24"
```

#### `Pose2D`
A 2D pose — position and heading.
```cpp
limelib::Pose2D pose(12.0f, 24.0f, 90.0f); // x=12", y=24", θ=90°
Pose2D inRad = pose.toRadians();
Pose2D inDeg = pose.toDegrees();
```

#### `LineSegment2D`
A directed segment between two `Point2D` endpoints.
```cpp
limelib::LineSegment2D seg({0, 0}, {48, 0});
```

#### `Ray2D`
An infinite ray from a starting point at a given angle in radians.
```cpp
limelib::Ray2D ray({0, 0}, M_PI / 2); // Points upward
```

---

### Tracking Wheels

`TrackingWheel` abstracts over either a **VEX Rotation Sensor** or a **motor-encoder differential** (averaging left/right motors).

```cpp
// Rotation-sensor based (preferred)
pros::Rotation rotSensor(15);
limelib::TrackingWheel verticalTW(&rotSensor, /*diameter=*/2.75, /*offset=*/-0.25);

// Motor-encoder based (no dedicated encoder)
limelib::TrackingWheel motorTW(&leftDT, &rightDT, /*diameter=*/3.25, /*gearRatio=*/1.0);
```

| Parameter | Description |
|---|---|
| `sensor` | Pointer to a `pros::Rotation` sensor (or `nullptr` for motor-based) |
| `diameter` | Wheel diameter in inches |
| `offset` | Lateral distance from robot center (inches). Positive = right of center |

Pass `nullptr` as the horizontal tracking wheel if your robot only has one encoder.

---

### Localization

Both localization classes implement the `Locator` interface:

```cpp
class Locator {
    virtual Pose2D update()    = 0;
    virtual void   calibrate() = 0;
    virtual Pose2D getPose(bool radians = false) const = 0;
    virtual void   setPose(Pose2D pose, bool radians = false) = 0;
    virtual void   setPose(real_t x, real_t y, real_t theta, bool radians = false) = 0;
    virtual Velocity getVelocity() const = 0;
};
```

Switching between `Odometry` and `MCL` requires only changing the declaration of `locator` — the `Chassis` class accepts either.

---

#### Odometry

Standard arc-based 2D odometry fusing tracking wheel distances with IMU heading.

```cpp
limelib::Odometry locator(&verticalTW, &horizontalTW, inertial);
//  or with no horizontal wheel:
limelib::Odometry locator(&verticalTW, nullptr, inertial);
```

**Constructor parameters:**

| Parameter | Type | Description |
|---|---|---|
| `verticalTW` | `TrackingWheel*` | Vertical (forward-facing) tracking wheel |
| `horizontalTW` | `TrackingWheel*` | Horizontal (sideways) tracking wheel, or `nullptr` |
| `imu` | `pros::IMU&` | VEX Inertial Sensor |
| `shouldTaskRun` | `bool` | Start a background PROS task that calls `update()` every 20 ms (default: `true`) |

When `shouldTaskRun = true`, the background task is started automatically during `calibrate()`. Pose and velocity are then always up to date with no manual polling required.

---

#### Monte Carlo Localization (MCL)

MCL models the robot's position as a distribution of weighted **particles**. Each particle is a candidate pose. On every update cycle, particles are:

1. **Propagated** forward using the latest odometry delta (with configurable noise).
2. **Weighted** by comparing predicted ray-cast distances against actual sensor readings.
3. **Resampled** — particles with higher weights survive; low-weight particles are replaced.

This continuously corrects accumulated odometry drift toward the globally consistent sensor-to-wall distances.

```cpp
limelib::MCL locator(
    &verticalTW,       // Vertical tracking wheel
    nullptr,           // Horizontal tracking wheel (or nullptr)
    inertial,          // IMU
    mclSensors,        // std::vector<MCLDistance>
    field,             // Field2D
    200,               // Number of particles (more = more accurate, more CPU)
    0.1f,              // Translation noise
    false,             // Debug mode (draws particles on VEX screen)
    5                  // Update intensity (lower = more frequent MCL updates)
);
```

**Constructor parameters:**

| Parameter | Type | Description |
|---|---|---|
| `verticalTW` | `TrackingWheel*` | Vertical tracking wheel |
| `horizontalTW` | `TrackingWheel*` | Horizontal tracking wheel, or `nullptr` |
| `imu` | `pros::Imu&` | VEX Inertial Sensor |
| `sensors` | `std::vector<MCLDistance>&` | Distance sensors attached to the robot |
| `field` | `Field2D&` | Field model including any obstacles |
| `num_particles` | `int` | Number of particles in the filter |
| `translationNoise` | `real_t` | Gaussian noise added to particle translation during propagation |
| `debug` | `bool` | Display particle cloud on the V5 Brain screen |
| `intensitivity` | `int` | Re-run MCL every N odometry cycles (default: `10`) |
| `shouldTaskRun` | `bool` | Start background PROS task |

**MCLDistance — sensor wrapper:**

Each `MCLDistance` binds a `pros::Distance` sensor to a robot-relative `Pose2D` describing where on the robot the sensor is mounted.

```cpp
// Pose2D(forward_offset, lateral_offset, heading_degrees)
limelib::MCLDistance frontSensor{frontDS, Pose2D(4.5f, 3.0f, 0.0f)};
```

| Pose component | Meaning |
|---|---|
| `x` | Distance forward from robot center (inches, positive = forward) |
| `y` | Distance sideways from robot center (inches, positive = right) |
| `theta` | Sensor ray direction in degrees, relative to robot heading |

**Tuning guidelines:**

- Start with `num_particles = 100–200`. Increase for higher accuracy at the cost of CPU.
- `translationNoise` between `0.05` and `0.2` works for most robots. Too low and MCL won't recover from odometry error; too high and the particle cloud disperses.
- Set `intensitivity = 5–15`. Lower values run MCL more frequently at higher CPU cost.
- Enable `debug = true` during tuning to visualize the particle cloud on the V5 screen.

---

### Motion Control

#### Chassis

`Chassis` wraps `moveToPoint`, `moveToPose`, `turnToHeading`, and `turnToPoint` into asyncronous PROS tasks with timeout safety.

```cpp
limelib::Chassis chassis(locator, leftDT, rightDT, linearPID, angularPID);
```

All motion functions block until the movement completes or times out.

---

##### `moveToPoint`

Drives to a target `(x, y)` coordinate using angular-priority control.

```cpp
chassis.moveToPoint(24, 48, 2000);                        // x, y, timeout_ms
chassis.moveToPoint(Point2D(24, 48), 2000);               // Point2D overload

// With custom parameters:
limelib::moveToPointParams params;
params.forwards         = true;   // Travel forward (false = reverse)
params.maxSpeed         = 100;    // Max motor output (0–127)
params.minSpeed         = 10;     // Min speed for motion chaining
params.earlyExitRange   = 2.0;    // Exit within this many inches of target
params.settleDistance   = 7.5;    // Disable turning when within this distance
params.driftCompensation = 2.0;   // 2 for omni wheels, 8 for traction wheels
params.slew             = 5;      // Max acceleration per cycle (0 = disabled)
chassis.moveToPoint(24, 48, 2000, params);
```

---

##### `moveToPose`

Drives to a full pose `(x, y, θ)`, aligning the robot's heading on arrival.

```cpp
chassis.moveToPose(24, 48, 90, 3000);                     // x, y, heading, timeout_ms

limelib::moveToPoseParams params;
params.lateralForwards  = true;
params.angularForwards  = true;
params.maxSpeed         = 127;
params.minSpeed         = 0;
params.earlyExitRange   = 0.0;
chassis.moveToPose(24, 48, 90, 3000, params);
```

---

##### `turnToHeading`

Turns in-place to an absolute heading.

```cpp
chassis.turnToHeading(90, 1500);                          // degrees, timeout_ms

limelib::turnToHeadingParams params;
params.maxSpeed       = 80;
params.earlyExitRange = 1.0;
chassis.turnToHeading(90, 1500, params);
```

---

##### `turnToPoint`

Turns to face a target `(x, y)` coordinate.

```cpp
chassis.turnToPoint(48, 72, 1500);
chassis.turnToPoint(Point2D(48, 72), 1500);
```

---

##### Other chassis utilities

```cpp
chassis.calibrate();                    // Calibrate all sensors and start localization task
chassis.setPose(-24, -24, 0);           // Set current position (degrees)
chassis.setPose(Pose2D(-24, -24, 0));   // Pose2D overload
chassis.cancelAllMovement();            // Abort current motion
chassis.waitUntilDone();                // Block until current motion completes
chassis.setPID(newLinear, newAngular);  // Swap PID controllers at runtime
```

---

#### PID Controller

A standard PID controller with optional integral windup limiting and automatic integral reset on error sign change.

```cpp
limelib::PID pid(
    3.0f,   // kP
    0.0f,   // kI
    2.0f,   // kD
    5.0f,   // windupRange — integral only accumulates when |error| < windupRange (0 = disabled)
    true    // signFlipReset — reset integral when error crosses zero
);

real_t output = pid.update(error);
pid.reset();
```

---

#### Trapezoidal Motion Profiling

`TrapezoidalMotionProfile` generates smooth speed profiles with configurable (and asymmetric) acceleration and deceleration.

```cpp
limelib::TrapezoidalMotionProfile profile(
    48.0f,  // maxSpeed (inches/second)
    24.0f   // maxAccel (inches/second²)
);
profile.setMaxDecel(36.0f);   // Optional: separate decel limit

profile.generateProfile(72.0f); // Plan a 72" move

real_t t = 1.0f; // seconds into the move
real_t pos   = profile.getPosition(t);
real_t vel   = profile.getVelocity(t);
real_t accel = profile.getAcceleration(t);

if (profile.isComplete(t)) { /* done */ }
real_t totalTime = profile.getTotalTime();
```

---

### Field Representation

`Field2D` holds the dimensions of the competition field and any obstacle objects used by the MCL raycaster.

```cpp
// Empty field (field bounds act as the only walls)
limelib::Field2D field(144.0f, 144.0f);

// Field with obstacles
auto goalBox = std::make_shared<limelib::Rectangle2D>(60.0f, 0.0f, 24.0f, 24.0f);
auto post    = std::make_shared<limelib::Circle2D>(0.0f, 0.0f, 3.0f);
limelib::Field2D field(144.0f, 144.0f, {goalBox, post});

// Modify obstacles at runtime
field.addObject(std::make_shared<limelib::Line2D>(0, 72, 144, 72));
field.removeObject(goalBox);
```

**Available obstacle types:**

| Class | Constructor | Description |
|---|---|---|
| `Rectangle2D` | `(x, y, width, height, theta=0)` | Axis-aligned or rotated rectangle |
| `Circle2D` | `(x, y, radius)` | Circle, approximated as a polygon for raycasting |
| `Polygon2D` | `(vector<Point2D> corners)` | Arbitrary convex/concave polygon |
| `Line2D` | `(startX, startY, endX, endY)` | Single line segment obstacle |

---

## Configuration Reference

A complete robot configuration typically lives in a `globals.hpp` file:

```cpp
#pragma once
#include "limelib/limelib.hpp"
using namespace limelib;

// --- Motors ---
inline pros::MotorGroup leftDT({-1, -2, -3});
inline pros::MotorGroup rightDT({8, 9, 10});

// --- Sensors ---
inline pros::Rotation verticalSensor(15);
inline pros::Imu inertial(13);

// --- Distance sensors ---
inline pros::Distance frontDS(21), backDS(4), leftDS(5), rightDS(20);

// --- Tracking wheel ---
inline TrackingWheel verticalTW(&verticalSensor, /*diameter=*/2.75, /*offset=*/-0.25);

// --- MCL sensor array ---
// Pose2D(forward_offset_in, lateral_offset_in, heading_deg)
inline std::vector<MCLDistance> mclSensors = {
    {frontDS,  Pose2D( 4.50,  3.00,   0)},
    {backDS,   Pose2D(-3.50, -5.50, 180)},
    {rightDS,  Pose2D( 4.25, -2.25,  90)},
    {leftDS,   Pose2D(-4.25, -2.25, 270)},
};

// --- Field ---
inline Field2D field(144.0f, 144.0f); // Add obstacles as needed

// --- Locator (swap Odometry ↔ MCL here) ---
inline MCL locator(&verticalTW, nullptr, inertial, mclSensors, field,
                   200, 0.1f, false, 5);
// inline Odometry locator(&verticalTW, nullptr, inertial);

// --- Controllers ---
inline PID linearPID(3.0f, 0.0f, 2.0f);
inline PID angularPID(0.9f, 0.0f, 1.0f);

// --- Chassis ---
inline Chassis chassis(locator, leftDT, rightDT, linearPID, angularPID);
```

---

## Contributing

Contributions, bug reports, and feature requests are welcome. Please open an issue or pull request on GitHub.

When contributing code:
- Follow the existing naming conventions and `limelib` namespace structure.
- Keep `real_t` (`float`) throughout for memory consistency.
- Test on hardware — V5 Brain simulation differs from physical behavior.
