#include "limelib/locator.hpp"

limelib::Odometry::Odometry(TrackingWheel *verticalTW, TrackingWheel *horizontalTW, pros::IMU &imu, bool shouldTaskRun)
    : verticalTW(verticalTW), horizontalTW(horizontalTW), imu(imu), currentPose(0, 0, 0), prevPose(0, 0, 0), headingOffset(0), shouldTaskRun(shouldTaskRun), lastUpdateTime(pros::millis()), currentVelocity(), odomTask(nullptr), master(pros::E_CONTROLLER_MASTER)
{
}

void limelib::Odometry::calibrate()
{
    if (verticalTW != nullptr)
        verticalTW->calibrate();
    if (horizontalTW != nullptr)
        horizontalTW->calibrate();
    imu.reset(true);
    master.rumble(".");
    headingOffset = -imu.get_heading() * M_PI / 180;
    if (shouldTaskRun)
    {
        odomTask = std::make_unique<pros::Task>([this]()
                                                {
            while (true)
            {
                update();
                pros::delay(20);
            } });
    }
}

void limelib::Odometry::setPose(limelib::Pose2D pose, bool radians)
{
    if (radians)
    {
        pose = pose.toDegrees();
    }
    headingOffset = (-imu.get_rotation() + pose.theta) * M_PI / 180.0;
    currentPose = pose.toRadians();
    // std::cout << "Odometry pose set to X: " << currentPose.x << " Y: " << currentPose.y << " Theta (deg): " << currentPose.theta << "\n";
}

void limelib::Odometry::setPose(real_t x, real_t y, real_t theta, bool radians)
{
    setPose(limelib::Pose2D(x, y, theta), radians);
}
limelib::Pose2D limelib::Odometry::update()
{
    // Calculate time delta in seconds
    uint32_t currentTime = pros::millis();
    real_t dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    real_t IMUReading = imu.get_rotation();
    real_t heading = IMUReading * M_PI / 180 + headingOffset;
    real_t headingDiff = heading - currentPose.theta;
    real_t vertDist = verticalTW != nullptr ? verticalTW->getDistanceTravelled() + verticalTW->getOffset() * headingDiff / 2 : 0;
    real_t horDist = horizontalTW != nullptr ? horizontalTW->getDistanceTravelled() + horizontalTW->getOffset() * headingDiff / 2 : 0;

    real_t xChange = vertDist * sin(heading) + horDist * cos(heading);
    real_t yChange = vertDist * cos(heading) - horDist * sin(heading);

    currentPose.x += xChange;
    currentPose.y += yChange;
    currentPose.theta = heading;

    // Calculate velocity (inches per second)
    if (dt > 0.001f) // Avoid division by zero
    {
        currentVelocity.x = xChange / dt;
        currentVelocity.y = yChange / dt;
        currentVelocity.linear = std::sqrt(xChange * xChange + yChange * yChange) / dt;
        currentVelocity.angular = headingDiff / dt;
    }

    return Pose2D(xChange, yChange, headingDiff);
}
limelib::Pose2D limelib::Odometry::getPose(bool radians) const
{
    Pose2D currentPose = this->currentPose;
    while (currentPose.theta < 0)
        currentPose.theta += 2 * M_PI;
    while (currentPose.theta >= 2 * M_PI)
        currentPose.theta -= 2 * M_PI;
    return radians ? currentPose : currentPose.toDegrees();
}

limelib::Velocity limelib::Odometry::getVelocity() const
{
    return currentVelocity;
}

limelib::MCL::MCL(TrackingWheel *verticalTW, TrackingWheel *horizontalTW,
                  pros::Imu &imu, std::vector<MCLDistance> &sensors, Field2D &field, int num_particles, real_t translationNoise, bool debug,
                  int intensity, bool shouldTaskRun)
    : odomHelper(verticalTW, horizontalTW, imu, false), sensors(sensors), field(field), NUM_PARTICLES(num_particles),
      TRANSLATION_NOISE(translationNoise), debug(debug), INTENSITY(intensity), last_mcl_update(intensity), randomParticleCount(0), shouldTaskRun(shouldTaskRun), prevPose(0, 0, 0), lastUpdateTime(pros::millis()), currentVelocity(), debugCounter(0)
{
    // Initialize particles with random positions but heading will be set from odometry later
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        // Start with 0 heading, will be updated to odometry heading in first update
        // Initialize with uniform weight
        particles.push_back(MCLParticle(Pose2D(getRandomReal_t(-field.getWidth() / 2, field.getWidth() / 2), getRandomReal_t(-field.getHeight() / 2, field.getHeight() / 2), 0), 1.0 / NUM_PARTICLES));
    }
}

void limelib::MCL::calibrate()
{
    odomHelper.calibrate();
    // std::cout << "Field Edges: " << field.getEdges().size() << std::endl;
    // std::cout << "MCL::calibrate - shouldTaskRun=" << shouldTaskRun << std::endl;
    if (shouldTaskRun)
    {
        // std::cout << "MCL::calibrate - Creating MCL task..." << std::endl;
        // Create task and store in member variable so it persists
        mclTask = std::make_unique<pros::Task>([this]()
                                               {
            // std::cout << "MCL task started!" << std::endl;
            while (true)
            {
                update();
                pros::delay(10);
            } });
        // std::cout << "MCL::calibrate - MCL task created, ptr=" << mclTask.get() << std::endl;
    }
    else
    {
        // std::cout << "MCL::calibrate - Task NOT created (shouldTaskRun=false)" << std::endl;
    }
}

void limelib::MCL::setPose(limelib::Pose2D pose, bool radians)
{
    odomHelper.setPose(pose, radians);

    // Update actualPose to match the set pose
    real_t odomHeading = radians ? pose.theta : pose.theta * M_PI / 180; // Convert to radians if needed
    actualPose.x = pose.x;
    actualPose.y = pose.y;
    actualPose.theta = odomHeading;

    // Reset odomDelta since we're setting a new pose
    odomDelta = Pose2D(0, 0, 0);

    // Initialize estimated pose
    estimatedPose.point = actualPose;
    estimatedPose.weight = 1.0; // Full confidence initially

    particles.clear();
    // Initialize particles with position spread but all using the same odometry heading
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        particles.push_back(MCLParticle(Pose2D(pose.x + getRandomReal_t(-4, 4), pose.y + getRandomReal_t(-4, 4), odomHeading), 1.0 / NUM_PARTICLES));
    }
}

void limelib::MCL::setPose(real_t x, real_t y, real_t theta, bool radians)
{
    setPose(limelib::Pose2D(x, y, theta), radians);
}

void limelib::MCL::setNoise(real_t translationNoise)
{
    this->TRANSLATION_NOISE = translationNoise;
}

limelib::Pose2D limelib::MCL::update()
{
    Pose2D currentDelta = odomHelper.update();
    odomDelta.x += currentDelta.x;
    odomDelta.y += currentDelta.y;
    odomDelta.theta += currentDelta.theta;
    // Normalize theta to [0, 2π)
    while (odomDelta.theta < 0)
        odomDelta.theta += 2 * M_PI;
    while (odomDelta.theta >= 2 * M_PI)
        odomDelta.theta -= 2 * M_PI;
    if (last_mcl_update > INTENSITY)
    {
        debugCounter++;
        updateMCL();
        if (debug)
            debugDisplay();
        last_mcl_update = 0;
    }
    else
    {
        last_mcl_update++;
    }

    currentVelocity = odomHelper.getVelocity();

    return odomDelta;
}

void limelib::MCL::updateMCL()
{
    // Get current odometry heading for all particles
    real_t odomHeading = odomHelper.getPose(true).theta;

    // Pre-generate random noise once per particle (not per coordinate)
    for (MCLParticle &particle : particles)
    {
        // Generate noise values once
        real_t noise_x = getRandomReal_t(-TRANSLATION_NOISE, TRANSLATION_NOISE);
        real_t noise_y = getRandomReal_t(-TRANSLATION_NOISE, TRANSLATION_NOISE);
        real_t noise_theta = getRandomReal_t(-0.1, 0.1);

        // Apply motion with noise
        particle.point.x += odomDelta.x + noise_x;
        particle.point.y += odomDelta.y + noise_y;
        particle.point.theta = odomHeading + noise_theta;
    }
    // 2. CORRECTION STEP - Update particle weights based on sensor observations
    // Pre-filter valid sensors to avoid checking in the hot loop

    std::vector<ValidSensor> validSensors;
    validSensors.reserve(sensors.size());

    // bool firstSensor = true;
    for (const MCLDistance &sensor : sensors)
    {
        int32_t raw = sensor.sensor.get_distance();
        int16_t size = sensor.sensor.get_object_size();

        // if (firstSensor)
        // {
        // pros::lcd::print(0, "DS1: %d mm, %d sz", raw, size);
        //     firstSensor = false;
        // }

        // Valid readings: positive, < 2000mm, size > 70
        if (raw > 0 && raw < 2000 && size > 70)
        {
            validSensors.push_back({
                static_cast<real_t>(raw),
                sensor.pose.x,
                sensor.pose.y,
                sensor.pose.theta * M_PI / 180.0 // Pre-convert to radians
            });
        }
    }

    // If no valid sensors, skip correction step - just apply odometry
    if (validSensors.size() == 0)
    {
        actualPose.x += odomDelta.x;
        actualPose.y += odomDelta.y;
        actualPose.theta = odomHelper.getPose(true).theta;
        odomDelta = Pose2D(0, 0, 0);
        return;
    }

    real_t totalWeight = 0.0;
    const real_t INV_2_VARIANCE = 1.0 / (2.0 * 16.0); // Pre-compute constant

    for (MCLParticle &particle : particles)
    {
        // Cache trig calculations (same for all sensors on this particle)
        const real_t cosTheta = cos(particle.point.theta);
        const real_t sinTheta = sin(particle.point.theta);

        real_t likelihood = 1.0;

        // For each valid sensor, compare actual vs expected readings
        for (const ValidSensor &sensor : validSensors)
        {
            const real_t actualDistance = sensor.reading * MM_TO_IN;

            // Transform sensor LOCAL position to GLOBAL position
            const real_t sensorGlobalX = particle.point.x + sensor.pose_x * cosTheta + sensor.pose_y * sinTheta;
            const real_t sensorGlobalY = particle.point.y + sensor.pose_y * cosTheta - sensor.pose_x * sinTheta;

            // Ray direction = robot heading + sensor facing angle (already in radians)
            const real_t sensorGlobalTheta = particle.point.theta + sensor.pose_theta_rad;

            // Cast ray from sensor position in sensor direction
            Ray2D ray(Point2D(sensorGlobalX, sensorGlobalY), sensorGlobalTheta);
            const real_t expectedDistance = getRayCastDistance(field.getEdges(), ray);

            // Calculate likelihood based on error
            const real_t error = std::abs(actualDistance - expectedDistance);
            const real_t errorSquared = error * error;
            real_t sensorLikelihood = exp(-errorSquared * INV_2_VARIANCE);

            // Apply minimum likelihood floor
            if (sensorLikelihood < 0.001)
                sensorLikelihood = 0.001;

            likelihood *= sensorLikelihood;
        }

        particle.weight = likelihood;
        totalWeight += likelihood;
    }
    // Normalize weights
    if (totalWeight > 0)
    {
        for (MCLParticle &particle : particles)
        {
            particle.weight /= totalWeight;
        }
    }
    else
    {
        // If all weights are zero, reset to uniform distribution
        for (MCLParticle &particle : particles)
        {
            particle.weight = 1.0 / NUM_PARTICLES;
        }
    }

    // 3. UPDATE POSE ESTIMATE - Calculate weighted average for position, use odometry for heading
    // (Must happen BEFORE resampling to use meaningful weights from correction step)
    real_t sumX = 0, sumY = 0, sumWeights = 0;
    real_t maxWeight = 0;

    for (const MCLParticle &particle : particles)
    {
        real_t weight = particle.weight;

        sumX += particle.point.x * weight;
        sumY += particle.point.y * weight;
        sumWeights += weight;
        maxWeight = std::max(maxWeight, weight);
    }
    // std::cout << "Max Particle Weight: " << maxWeight << std::endl;
    if (sumWeights > 0)
    {
        // Raw MCL estimate from weighted average
        real_t rawMCL_x = sumX / sumWeights;
        real_t rawMCL_y = sumY / sumWeights;

        // Apply exponential smoothing to reduce oscillation (alpha = 0.3 means 70% previous, 30% new)
        const real_t SMOOTHING_ALPHA = 0.8;
        estimatedPose.point.x = SMOOTHING_ALPHA * rawMCL_x + (1.0 - SMOOTHING_ALPHA) * estimatedPose.point.x;
        estimatedPose.point.y = SMOOTHING_ALPHA * rawMCL_y + (1.0 - SMOOTHING_ALPHA) * estimatedPose.point.y;
        // Use odometry heading directly instead of particle-based rotation
        estimatedPose.point.theta = odomHeading;

        // Calculate confidence based on particle concentration using effective sample size
        real_t ess = 1.0;
        real_t sumSquaredWeights = 0.0;
        for (const MCLParticle &p : particles)
        {
            sumSquaredWeights += p.weight * p.weight;
        }
        if (sumSquaredWeights > 0)
        {
            ess = 1.0 / sumSquaredWeights;
        }
        // Normalize ESS to [0, 1] range and apply smoothing
        real_t essConfidence = ess / NUM_PARTICLES;
        estimatedPose.weight = SMOOTHING_ALPHA * essConfidence + (1.0 - SMOOTHING_ALPHA) * estimatedPose.weight;

        // Use confidence to blend MCL estimate with odometry
        real_t confidence = estimatedPose.weight;
        confidence = std::clamp(confidence, 0.1f, 0.85f); // More conservative blending to reduce jumps

        // Calculate odometry-based pose from last actualPose + accumulated delta
        Pose2D odomBasedPose;
        odomBasedPose.x = actualPose.x + odomDelta.x;
        odomBasedPose.y = actualPose.y + odomDelta.y;
        odomBasedPose.theta = odomHeading; // Use current odometry heading

        // Blend only X and Y positions based on confidence, use odometry heading directly
        actualPose.x = confidence * estimatedPose.point.x + (1.0 - confidence) * odomBasedPose.x;
        actualPose.y = confidence * estimatedPose.point.y + (1.0 - confidence) * odomBasedPose.y;
        // Always use odometry heading (no rotation correction)
        actualPose.theta = odomHeading;
    }
    else
    {
        // If sumWeights is 0, just apply odometry
        actualPose.x += odomDelta.x;
        actualPose.y += odomDelta.y;
        actualPose.theta = odomHeading;
    }

    // 4. RESAMPLING STEP - Resample particles based on weights
    std::vector<MCLParticle> newParticles;
    newParticles.reserve(NUM_PARTICLES);

    // Calculate how many random particles to inject (for kidnapping resistance)
    int numRandom = randomParticleCount;
    int numResampled = NUM_PARTICLES - numRandom;

    // Guard against edge cases
    if (numResampled < 0)
        numResampled = 0;
    if (numRandom > NUM_PARTICLES)
        numRandom = NUM_PARTICLES;

    // Only do systematic resampling if we have particles to resample
    if (numResampled > 0)
    {
        // Resample particles using systematic resampling
        real_t step = 1.0 / numResampled;
        real_t r = getRandomReal_t(0, step);
        real_t c = particles[0].weight;
        int i = 0;

        for (int m = 0; m < numResampled; m++)
        {
            real_t u = r + m * step;
            while (u > c && i < NUM_PARTICLES - 1)
            {
                i++;
                c += particles[i].weight;
            }
            MCLParticle resampled = particles[i];
            resampled.weight = 1.0 / NUM_PARTICLES; // Reset weight after resampling
            newParticles.push_back(resampled);
        }
    }

    // Add random particles for kidnapping resistance (using current odometry heading)
    for (int j = 0; j < numRandom; j++)
    {
        real_t randomX = getRandomReal_t(-field.getWidth() / 2, field.getWidth() / 2);
        real_t randomY = getRandomReal_t(-field.getHeight() / 2, field.getHeight() / 2);
        // Use current odometry heading instead of random rotation
        newParticles.push_back(MCLParticle(Pose2D(randomX, randomY, odomHeading), 1.0 / NUM_PARTICLES));
    }

    particles = std::move(newParticles);
    odomDelta = Pose2D(0, 0, 0); // Reset odomDelta after update
}

limelib::Pose2D limelib::MCL::getPose(bool radians) const
{
    // If we've never run MCL or have no valid estimate, fall back to pure odometry
    if (estimatedPose.weight <= 0)
    {
        return odomHelper.getPose(radians);
    }

    Pose2D blendedPose;
    blendedPose.x = actualPose.x + odomDelta.x;
    blendedPose.y = actualPose.y + odomDelta.y;
    blendedPose.theta = odomHelper.getPose(radians).theta;
    return blendedPose;
}

limelib::Velocity limelib::MCL::getVelocity() const
{
    return currentVelocity;
}

void limelib::MCL::debugDisplay()
{
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::white);
    int height = field.getHeight();
    int width = field.getWidth();
    pros::screen::draw_rect(10, 10, width + 10, height + 10);
    for (const auto &edge : field.getEdges())
    {
        int x0 = static_cast<int>(edge.start.x) + 10 + width / 2;
        int y0 = static_cast<int>(edge.start.y) + 10 + height / 2;
        int x1 = static_cast<int>(edge.end.x) + 10 + width / 2;
        int y1 = static_cast<int>(edge.end.y) + 10 + height / 2;
        pros::screen::draw_line(x0, y0, x1, y1);
    }
    pros::screen::set_pen(pros::Color::blue);
    for (const auto &particle : particles)
    {
        int px = static_cast<int>(particle.point.x) + 10 + width / 2;
        int py = static_cast<int>(particle.point.y) + 10 + height / 2;
        pros::screen::draw_circle(px, py, 1);
    }
    int ax = static_cast<int>(actualPose.x) + 10 + width / 2;
    int ay = static_cast<int>(actualPose.y) + 10 + height / 2;
    pros::screen::set_pen(pros::Color::red);
    pros::screen::draw_circle(ax, ay, 2);
}

limelib::real_t limelib::getRayCastDistance(const std::vector<LineSegment2D> &edges, Ray2D ray)
{
    real_t min_distance = 1000.0;

    // Pre-calculate ray direction
    const real_t ray_dx = std::sin(ray.radians);
    const real_t ray_dy = std::cos(ray.radians);
    const real_t EPSILON = 1e-9;

    for (const auto &edge : edges)
    {
        // Get line segment vectors
        const real_t dx = edge.end.x - edge.start.x;
        const real_t dy = edge.end.y - edge.start.y;

        // Calculate determinant for intersection
        const real_t det = ray_dx * dy - ray_dy * dx;

        // Skip if parallel (use direct comparison to avoid abs() call)
        if (det > -EPSILON && det < EPSILON)
            continue;

        // Vector from ray start to line segment start
        const real_t px = edge.start.x - ray.start.x;
        const real_t py = edge.start.y - ray.start.y;

        // Calculate intersection parameters
        const real_t inv_det = 1.0 / det;
        const real_t u = (ray_dy * px - ray_dx * py) * inv_det;

        // Early exit if u is out of bounds
        if (u < 0.0 || u > 1.0)
            continue;

        const real_t t = (dy * px - dx * py) * inv_det;

        // Check if intersection is in front and closer than current minimum
        if (t > EPSILON && t < min_distance)
        {
            min_distance = t;
        }
    }

    return min_distance;
}
