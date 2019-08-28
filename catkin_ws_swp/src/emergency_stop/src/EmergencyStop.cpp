#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig &config) { this->config = config; }

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {

        if (emergencyStop == false) {
            safeSpeedPWM = wantedSpeed;
        }
        emergencyStop = false;

        if (wantedSpeed < 0.0) {
            direction = Direction::BACKWARD;
        } else if (wantedSpeed > 0.0) {
            direction = Direction::FORWARD;
        }

        if (config.steering_angle_tolerance >= currentSteeringAngle &&
            currentSteeringAngle >= -config.steering_angle_tolerance) {
            steering = Steering::STRAIGHT;
        }
        else{
            if (currentSteeringAngle < -config.steering_angle_tolerance) {
                steering = Steering::RIGHT;
            }
            else{
                steering = Steering::LEFT;
            }
            turningRadius = getTurningRadius(currentSteeringAngle);
            // inner rear tire
            turningRadiusIR = turningRadius - (config.car_width / 2); //turningRadius - (config.track / 2);
            // outer front edge
            turningRadiusOF = hypotenuse(turningRadiusIR + config.car_width,
                                         config.lidar_rear_axle_distance + config.forward_minimum_distance);
        }

        //static_cast<double>(direction)


        auto stopDistance = config.stop_distance;
        auto targetQuotient = config.target_quotient;
        auto negAcc = config.negative_acceleration;
        auto maxRange = config.maximum_lidar_radius;
        auto frontAngle = config.half_angle_front_init;
        auto backAngle = config.half_angle_back_init;
        // include rear tire when turning while driving forward:
        auto fwdTurningBackAngle = asin(config.lidar_rear_axle_distance / std::sqrt(
                std::pow(config.car_width / 2, 2) + std::pow(config.lidar_rear_axle_distance, 2)));

        // init minDist
        auto minDistance = std::numeric_limits<double>::infinity();
        auto angleIncrement = scan->angle_increment;

        // nearest object to car
        obstacleDistanceActual = std::numeric_limits<double>::infinity();
        for (int i = 0; i < scan->ranges.size(); i++) {
            double tmp = getExactDistanceToCar(i*angleIncrement, scan->ranges[i]);
            if(tmp > 0 && tmp < obstacleDistanceActual) {
                obstacleDistanceActual = tmp;
            }
        }

        if(direction == Direction::FORWARD) {

            if(steering == Steering::STRAIGHT) {
                steers = "straight";

                auto start = 0;
                auto end = static_cast<int>(frontAngle / angleIncrement);
                for (int i = start; i < scan->ranges.size() && i < end; i++) {
                    auto dist = getStraightDistanceToCar(scan->ranges[i], i);
                    if (dist < maxRange && dist > 0 && dist < minDistance && isOnStraightPath(i*angleIncrement, scan->ranges[i], false)) {
                        minDistance = dist;
                    }
                }

                start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
                end = scan->ranges.size();
                for (int k = start; k < end; k++) {
                    auto dist = getStraightDistanceToCar(scan->ranges[k], k);
                    if (dist < maxRange && dist > 0 && dist < minDistance && isOnStraightPath((360-k)*angleIncrement, scan->ranges[k], false)) {
                        minDistance = dist;
                    }
                }
            }

            if (steering == Steering::LEFT) {
                steers = "left";

                auto start = 0;
                auto end = scan->ranges.size() / 2 - static_cast<int>((DEG90INRAD - fwdTurningBackAngle) / angleIncrement);
                for (int i = start; i < scan->ranges.size() && i < end; i++) {
                    auto alpha = i * angleIncrement;
                    auto distance = scan->ranges[i];
                    if (distance < maxRange && !isOnCar(alpha, distance)) {
                        auto dist = getDistanceToCarOnPath(alpha, distance, turningRadius, turningRadiusIR, turningRadiusOF, direction, steering);
                        if (dist < minDistance) { // && dist >= 0
                            minDistance = dist;
                        }
                    }
                }

                start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
                end = scan->ranges.size();
                for (int k = start; k < end; k++) {
                    auto alpha = k * angleIncrement;
                    auto distance = scan->ranges[k];
                    if (distance < maxRange && !isOnCar(alpha, distance)) {
                        auto dist = getDistanceToCarOnPath(alpha, distance, turningRadius, turningRadiusIR, turningRadiusOF, direction, steering);
                        if (dist < minDistance) { // && dist >= 0
                            minDistance = dist;
                        }
                    }
                }
            }

            if(steering == Steering::RIGHT) {
                steers = "right";

                auto start = 0;
                auto end = static_cast<int>(frontAngle / angleIncrement);
                for (int i = start; i < scan->ranges.size() && i < end; i++) {
                    auto alpha = i * angleIncrement;
                    auto distance = scan->ranges[i];
                    if (distance < maxRange && !isOnCar(alpha, distance)) {
                        auto dist = getDistanceToCarOnPath(alpha, distance, turningRadius, turningRadiusIR, turningRadiusOF, direction, steering);
                        if (dist < minDistance) { // && dist >= 0
                            minDistance = dist;
                        }
                    }
                }

                start = scan->ranges.size() / 2 + static_cast<int>((DEG90INRAD - fwdTurningBackAngle) / angleIncrement);
                end = scan->ranges.size();
                for (int k = start; k < end; k++) {
                    auto alpha = k * angleIncrement;
                    auto distance = scan->ranges[k];
                    if (distance < maxRange && !isOnCar(alpha, distance)) {
                        auto dist = getDistanceToCarOnPath(alpha, distance, turningRadius, turningRadiusIR, turningRadiusOF, direction, steering);
                        if (dist < minDistance) { // && dist >= 0
                            minDistance = dist;
                        }
                    }
                }
            }


        } else { // direction == Direction::BACKWARD

            if(steering == Steering::STRAIGHT) {
                steers = "straight";
                // back left to back right
                int start = scan->ranges.size() / 2 - static_cast<int>(backAngle / angleIncrement);
                int end = scan->ranges.size() / 2 + static_cast<int>(backAngle / angleIncrement);
                for (int j = start; j < end && j < scan->ranges.size(); j++) {
                    auto dist = getStraightDistanceToCar(scan->ranges[j], j);
                    if (dist < maxRange && dist > 0 && dist < minDistance && isOnStraightPath((abs(j-180))*angleIncrement, scan->ranges[j], true)) {
                        minDistance = dist;
                    }
                }
            }

            if(steering == Steering::LEFT) {
                steers = "left";

                int start = scan->ranges.size() / 4;
                int end = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
                for (int j = start; j < end && j < scan->ranges.size(); j++) {
                    auto alpha = j * angleIncrement;
                    auto distance = scan->ranges[j];
                    if (distance < maxRange && !isOnCar(alpha, distance)) {
                        auto dist = getDistanceToCarOnPath(alpha, distance, turningRadius, turningRadiusIR, turningRadiusOF, direction, steering);
                        if (dist < minDistance) { // && dist >= 0
                            minDistance = dist;
                        }
                    }
                }
            }

            if(steering == Steering::RIGHT) {
                steers = "right";

                int start = static_cast<int>(frontAngle / angleIncrement);
                int end = 3 * scan->ranges.size() / 4;
                for (int j = start; j < end && j < scan->ranges.size(); j++) {
                    auto alpha = j * angleIncrement;
                    auto distance = scan->ranges[j];
                    if (distance < maxRange && !isOnCar(alpha, distance)) {
                        auto dist = getDistanceToCarOnPath(alpha, distance, turningRadius, turningRadiusIR, turningRadiusOF, direction, steering);
                        if (dist < minDistance) { // && dist >= 0
                            minDistance = dist;
                        }
                    }
                }
            }
        }

        obstacleDistance = minDistance;

        // EMERGENCY STOP LOGIC:
        // based on speed, distance and deceleration, decide if EmergencyStop
        if (minDistance <= stopDistance) {
            safeSpeedPWM = 0;
            emergencyStop = true;
            return;
        }
        if (safeDistanceQuotient(minDistance, negAcc, currentSpeed) <= targetQuotient) {
            safeSpeedSI = calculateSafeSpeed(minDistance, negAcc, targetQuotient);
            auto speedQuotient = safeSpeedSI / currentSpeed;
            safeSpeedPWM = static_cast<int16_t>(safeSpeedPWM * speedQuotient);
            emergencyStop = true;
            return;
        }
    }

    /*
     *  Calculations
     */

    double EmergencyStop::hypotenuse(double a, double b) {
        return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
    }

    double EmergencyStop::getStraightDistanceToCar(double distanceToLidar, int deg_step) {
        if (deg_step < 90 || deg_step > 270) {
            return (distanceToLidar - config.forward_minimum_distance);
        } else {
            return (distanceToLidar - config.reverse_minimum_distance);
        }
    }

    double
    EmergencyStop::getDistanceToCarOnPath(double angle, double distance, double turningRadius, double turningRadiusIR,
                                          double turningRadiusOF, Direction direction, Steering steering) {
        // center -> center of turning circle
        double x_center, y_center, x_obstacle, y_obstacle, angle_corrected;
        // steering left  -> 1
        // steering right -> -1
        x_center = turningRadius * static_cast<double>(steering) * -1;
        y_center = 0;
        angle_corrected = getAlignedAngle(angle);
        std::tie(x_obstacle, y_obstacle) = polar2Cart(distance, angle_corrected);
        // project on rear axle (working with virtual lidar on middle of rear axle)
        y_obstacle += config.lidar_rear_axle_distance;

        // check distance from potential obstacle to center of turning circle
        double distanceFromCenter = getEuclideanDistance(x_center, y_center, x_obstacle, y_obstacle);

        if (isOnPath(distanceFromCenter, turningRadiusIR, turningRadiusOF)) {
            std::vector<double> vecCenter, vecObstacle;
            vecCenter[0] = 0 - x_center;
            vecCenter[1] = 0 - y_center;
            vecObstacle[0] = x_obstacle - x_center;
            vecObstacle[1] = y_obstacle - y_center;

            double angleOfDistance;
            if ((direction == Direction::FORWARD && steering == Steering::RIGHT) ||
                (direction == Direction::BACKWARD && steering == Steering::LEFT)) {
                angleOfDistance = getAngleBetweenVectors(vecCenter, vecObstacle);
            } else if ((direction == Direction::FORWARD && steering == Steering::LEFT) ||
                       (direction == Direction::BACKWARD && steering == Steering::RIGHT)) {
                angleOfDistance = getAngleBetweenVectors(vecObstacle, vecCenter);
            } else {
                return std::numeric_limits<double>::infinity();
            }

            auto angleOffset = calcOffsetAngle(distanceFromCenter, turningRadiusIR, direction);
            auto effectiveDistance = calcEffectiveDistance(angleOfDistance, angleOffset, distanceFromCenter);
            return effectiveDistance;
        }
        return std::numeric_limits<double>::infinity();
    }

    double EmergencyStop::calcEffectiveDistance(double angle, double angleOffset, double radius) {
        return (angle - angleOffset) * radius;
    }

    double EmergencyStop::getAngleBetweenVectors(std::vector<double> a, std::vector<double> b) {
        return fmod(atan2(a.at(1), a.at(0)) - atan2(b.at(1), b.at(0)), 2 * M_PI);
    }

    double EmergencyStop::calcOffsetAngleFront(double distance_from_center, Direction direction) {
        double numerator;
        if (direction == Direction::FORWARD) {
            numerator = config.forward_minimum_distance + config.lidar_rear_axle_distance;
        } else {
            numerator = config.reverse_minimum_distance - config.lidar_rear_axle_distance;
        }
        return asin(numerator / distance_from_center);
    }

    double EmergencyStop::calcOffsetAngleSide(double distance_from_center, double r_ir) {
        return acos(r_ir / distance_from_center);
    }

    double EmergencyStop::calcOffsetAngle(double distance_from_center, double r_ir, Direction direction) {
        return std::min(calcOffsetAngleSide(distance_from_center, r_ir),
                        calcOffsetAngleFront(distance_from_center, direction));
    }

    double EmergencyStop::getExactDistanceToCar(double rad, double dist) {
        // works with actual lidar, not with the projected!
        double offsetFront = config.forward_minimum_distance;
        double offsetRear = config.reverse_minimum_distance;
        double angleFront = config.half_angle_front_init;
        double angleRear = config.half_angle_back_init;
        double alpha;
        double a, b, c;

        // front: approx. same distance -> because of radial bumper
        if (angleFront >= rad || rad >= (DEG360INRAD - angleFront)) {
            return dist - offsetFront;
        }
        else if (angleFront < rad && rad <= DEG90INRAD) {
            alpha = DEG90INRAD - rad;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        else if (DEG90INRAD < rad && rad < (DEG180INRAD - angleRear)) {
            alpha = rad - DEG90INRAD;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        else if ((DEG180INRAD - angleRear) <= rad && rad < DEG180INRAD) {
            alpha = DEG180INRAD - rad;
            b = offsetRear;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        else if (DEG180INRAD <= rad && rad <= (DEG180INRAD + angleRear)) {
            alpha = rad - DEG180INRAD;
            b = offsetRear;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        else if ((DEG180INRAD + angleRear) < rad && rad <= DEG270INRAD) {
            alpha = DEG270INRAD - rad;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        else if (DEG270INRAD < rad && rad < (DEG360INRAD - angleFront)) {
            alpha = rad - DEG270INRAD;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        // on coding error:
        else {
            return std::numeric_limits<double>::infinity();
        }
    }

    bool EmergencyStop::isOnCar(double rad, double dist) {
        return getExactDistanceToCar(rad, dist) - config.dim_extension < 0;
    }

    double EmergencyStop::getTurningRadius(double steeringAngle) {
        return config.wheelbase / abs(tan(steeringAngle));
    }

    /*
     * rotate coordinate system, s.t. 0 degrees are straight ahead;
     * assumes that lidar is installed, s.t. 0 degrees are straight ahead
     */
    void EmergencyStop::alignCoordToFront(double &angle, double coordRot = DEG90INRAD) {
        tmp = fmod(angle + coordRot, 2 * M_PI);
        angle = tmp;
    }

    double EmergencyStop::getAlignedAngle(double angle, double coordRot = DEG90INRAD) {
        return fmod(angle + coordRot, 2 * M_PI);
    }

    std::tuple<double, double> EmergencyStop::polar2Cart(double r, double theta) {
        return std::make_tuple(r * cos(theta), r * sin(theta));
    }

    std::tuple<double, double> EmergencyStop::cart2Polar(double x, double y) {
        double theta = fmod(atan2(y, x), 2*M_PI);
        double r = hypotenuse(x, y);
        return std::make_tuple(r, theta);
    }

    double EmergencyStop::getEuclideanDistance(double x_center, double y_center, double x_obstacle, double y_obstacle) {
        return hypotenuse(x_center - x_obstacle, y_center - y_obstacle);
    }

    bool EmergencyStop::isOnPath(double x, double rIR, double rOF) {
        //auto correction = (config.car_width - config.track) / 2;
        return (x > (rIR - config.safety_margin / 2) &&  // - correction
                x < (rOF + config.safety_margin / 2)); // ToDo: figure out 0.01 exactly  // + correction + 0.01
    }

    double EmergencyStop::calculateSafeSpeed(double distance, double deceleration, double targetQuotient) {
        return std::sqrt(distance * deceleration / targetQuotient);
    }

    double EmergencyStop::safeDistanceQuotient(double distance, double deceleration, double currentSpeed) {
        if (currentSpeed == 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return distance * deceleration / std::pow(currentSpeed, 2);
    }

    // works with actual lidar position & values
    bool EmergencyStop::isOnStraightPath(double posAngle, double distance, bool backward) {
        auto distanceFromMiddle = sin(posAngle) * distance;
        auto safetyWidth = config.safety_margin;
//        if(backward){
//            safetyWidth = 0.04;
//        }
        return distanceFromMiddle < ((config.car_width + safetyWidth) / 2);
    }

    /*
     *  Setters for Subscribers
     */

    void EmergencyStop::setCurrentSpeed(const autominy_msgs::SpeedConstPtr &speed) {
        currentSpeed = speed->value;
    }

    void EmergencyStop::setCurrentSteeringAngle(const autominy_msgs::SteeringAngleConstPtr &steering) {
        this->currentSteeringAngle = steering->value;
    }

    void EmergencyStop::setWantedSpeed(const autominy_msgs::SpeedCommandConstPtr &speed) {
        wantedSpeed = speed->value;
    }


    /*
     *  Getters for Publishers
     */

    std_msgs::Float32 EmergencyStop::getCurrentTurningRadius() {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(turningRadius);
        return msg;
    }

    std_msgs::String EmergencyStop::getSteering() {
        std_msgs::String msg;
        msg.data = steers;
        return msg;
    }

    std_msgs::Float32 EmergencyStop::getSteeringAngleSub() {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(currentSteeringAngle);
        return msg;
    }

    std_msgs::Float32 EmergencyStop::getDistanceToObstacle() {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(obstacleDistance);
        return msg;
    }

    std_msgs::Float32 EmergencyStop::getDistanceFromCar() {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(obstacleDistanceActual);
        return msg;
    }

    autominy_msgs::SpeedCommand EmergencyStop::getSafeSpeed() {
        autominy_msgs::SpeedCommand msg;
        msg.value = safeSpeedPWM;
        return msg;
    }

    autominy_msgs::SpeedCommand EmergencyStop::getSpeedToPublish() {
        autominy_msgs::SpeedCommand msg;
        if (emergencyStop && abs(safeSpeedPWM) < abs(wantedSpeed)) {
            msg.value = safeSpeedPWM;
        } else {
            // dampen start up
            msg.value = wantedSpeed *
                        boost::algorithm::clamp(
                                obstacleDistance / (config.max_startup_damp_range * abs(wantedSpeed) / 1000), 0, 1);
            //boost::algorithm::clamp(obstacleDistance / (config.startup_damp_range + std::sqrt(wantedSpeed/1000)), 0, 1); // startup_damp_range = 1.0
            //std::pow(boost::algorithm::clamp(obstacleDistance / ((boost::algorithm::clamp(wantedSpeed, 200, 1000)/200) - 0.5), 0, 1), 2); // startup_damp_range = 1.0
        }
        return msg;
    }
}