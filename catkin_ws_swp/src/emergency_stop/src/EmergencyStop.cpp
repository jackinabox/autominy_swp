#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    //const double L = config.wheelbase;
    //const double T = config.track;
    //const double LENGTH = config.car_length;
    //const double WIDTH = config.car_width;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig &config) { this->config = config; }

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {

        if (emergencyStop == false) {
            safeSpeedPWM = wantedSpeed;
        }

        if (wantedSpeed < 0.0) {
            direction = Direction::BACKWARD;
        } else if (wantedSpeed > 0.0) {
            direction = Direction::FORWARD;
        }
        // * static_cast<double>(direction);

        //currentSteeringAngle = config.currentSteeringAngle;
        //double turningRadius;

        if (currentSteeringAngle < -config.steering_angle_tolerance) {
            orientation = Orientation::RIGHT;
            currentTurningRadius = getTurningRadius(currentSteeringAngle);
        } else if (currentSteeringAngle > config.steering_angle_tolerance) {
            orientation = Orientation::LEFT;
            currentTurningRadius = getTurningRadius(currentSteeringAngle);
        } else {
            orientation = Orientation::STRAIGHT;
        }

        double stopDistance = config.stop_distance;
        double targetQuotient = config.target_quotient;
        double negAcc = config.negative_acceleration;
        double maxRange = config.maximum_lidar_radius;

        emergencyStop = false;
        auto minDistance = std::numeric_limits<double>::infinity(); // init minDist
        auto angleIncrement = scan->angle_increment;

        auto frontAngle = config.half_angle_front_init;
        auto backAngle = config.half_angle_back_init;

        if(direction == Direction::FORWARD) {
            // DEPENDING ON STEERING DIRECTION

            if(orientation == Orientation::STRAIGHT) {
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

            if(orientation == Orientation::LEFT) {
                steers = "left";

                auto start = 0;
                auto end = scan->ranges.size() / 2 - static_cast<int>(frontAngle / angleIncrement);
                for (int i = start; i < scan->ranges.size() && i < end; i++) {
                    auto alpha = i * angleIncrement;
                    auto distance = scan->ranges[i];
                    auto projAlpha = alpha; //getProjectedAngle(alpha, distance, config.lidar_rear_axle_distance);
                    auto projDistance = distance; //getProjectedDistance(alpha, distance, config.lidar_rear_axle_distance);
                    auto x = getX(currentTurningRadius, projAlpha, projDistance);
                    if(isOnPath(currentTurningRadius, x)){
                        auto dist = getStraightDistanceToCar(scan->ranges[i], i);
                        if (dist < maxRange && dist > 0 && dist < minDistance) {
                            minDistance = dist;
                        }
                    }
                }

                start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
                end = scan->ranges.size();
                for (int k = start; k < end; k++) {

                    auto alpha = k * angleIncrement;
                    auto distance = scan->ranges[k];
                    auto projAlpha = alpha; //getProjectedAngle(alpha, distance, config.lidar_rear_axle_distance);
                    auto projDistance = distance; //getProjectedDistance(alpha, distance, config.lidar_rear_axle_distance);
                    auto x = getX(currentTurningRadius, projAlpha, projDistance);
                    if(isOnPath(currentTurningRadius, x)){
                        auto dist = getStraightDistanceToCar(scan->ranges[k], k);
                        if (dist < maxRange && dist > 0 && dist < minDistance) {
                            minDistance = dist;
                        }
                    }
                }
            }

            if(orientation == Orientation::RIGHT) {
                steers = "right";

                auto start = 0;
                auto end = static_cast<int>(frontAngle / angleIncrement);
                for (int i = start; i < scan->ranges.size() && i < end; i++) {
                    auto dist = getStraightDistanceToCar(scan->ranges[i], i);
                    if (dist < maxRange && dist > 0 && dist < minDistance) {
                        minDistance = dist;
                    }
                }

                start = scan->ranges.size() / 2 + static_cast<int>(frontAngle / angleIncrement);
                end = scan->ranges.size();
                for (int k = start; k < end; k++) {
                    auto dist = getStraightDistanceToCar(scan->ranges[k], k);
                    if (dist < maxRange && dist > 0 && dist < minDistance) {
                        minDistance = dist;
                    }
                }
            }


        } else { // direction == Direction::BACKWARD
            // DEPENDING ON STEERING DIRECTION

            if(orientation == Orientation::STRAIGHT) {
                steers = "straight";
                // back left to back right
                int start = scan->ranges.size() / 2 - static_cast<int>(backAngle / angleIncrement);
                int end = scan->ranges.size() / 2 + static_cast<int>(backAngle / angleIncrement);
                for (int j = start; j < end && j < scan->ranges.size(); j++) {
                    // we might see the camera in the laser scan
                    auto dist = getStraightDistanceToCar(scan->ranges[j], j);
                    if (dist < maxRange && dist > 0 && dist < minDistance && isOnStraightPath((abs(j-180))*angleIncrement, scan->ranges[j], true)) {
                        minDistance = dist;
                    }
                }
            }

            if(orientation == Orientation::LEFT) {
                steers = "left";

                int start = scan->ranges.size() / 4;
                int end = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
                for (int j = start; j < end && j < scan->ranges.size(); j++) {
                    // we might see the camera in the laser scan
                    auto dist = getStraightDistanceToCar(scan->ranges[j], j);
                    if (dist < maxRange && dist > 0 && dist < minDistance) {
                        minDistance = dist;
                    }
                }
            }

            if(orientation == Orientation::RIGHT) {
                steers = "right";

                int start = static_cast<int>(frontAngle / angleIncrement);
                int end = 3 * scan->ranges.size() / 4;
                for (int j = start; j < end && j < scan->ranges.size(); j++) {
                    // we might see the camera in the laser scan
                    auto dist = getStraightDistanceToCar(scan->ranges[j], j);
                    if (dist < maxRange && dist > 0 && dist < minDistance) {
                        minDistance = dist;
                    }
                }
            }
        }

        obstacleDistance = minDistance;
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

    double EmergencyStop::projectOnRearAxleAngle(double angle, double distance, double offset=config.lidar_rear_axle_distance) {

        double a;
        double b;

        if (angle >= 0.0 and angle < DEG90INRAD) { // 0 - 90 deg
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            return atan(a / (b + offset));
        }

        if (angle >= DEG90INRAD and angle < DEG180INRAD) { // 90 - 180 deg
            angle = DEG180INRAD - angle;
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            if (b > offset) {
                return (DEG180INRAD - (atan(a / (b - offset))));
            }
            else if (b < offset) {
                return atan(a / (offset - b));
            }
            else {
                return DEG90INRAD;
            }
        }

        if (angle >= DEG180INRAD and angle < DEG270INRAD) { // 180 - 270 deg
            angle = angle - DEG180INRAD;
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            if (b > offset) {
                return (DEG180INRAD + atan(a / (b - offset)));
            }
            else if (b < offset) {
                return (DEG360INRAD - atan(a / (offset - b)));
            }
            else {
                return DEG270INRAD;
            }
        }

        if (angle >= DEG270INRAD and angle < DEG360INRAD) { // 270 - 360 deg
            angle = DEG360INRAD - angle;
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            return (DEG360INRAD - atan(a / (b + offset)));
        }
    }

    double EmergencyStop::projectOnRearAxleDist(double angle, double distance, double offset=config.lidar_rear_axle_distance) {

        double a;
        double b;

        if (angle >= 0.0 and angle < DEG90INRAD) { // 0 - 90 deg
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            return std::sqrt(std::pow(a, 2) + std::pow(b + offset, 2));
        }

        if (angle >= DEG90INRAD and angle < DEG180INRAD) { // 90 - 180 deg
            angle = DEG180INRAD - angle;
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            if (b > offset) {
                return std::sqrt(std::pow(a, 2) + std::pow(b - offset, 2));
            }
            else if (b < offset) {
                return std::sqrt(std::pow(a, 2) + std::pow(offset - b, 2));
            }
            else {
                return a;
            }
        }

        if (angle >= DEG180INRAD and angle < DEG270INRAD) { // 180 - 270 deg
            angle = angle - DEG180INRAD;
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            if (b > offset) {
                return std::sqrt(std::pow(a, 2) + std::pow(b - offset, 2));
            }
            else if (b < offset) {
                return std::sqrt(std::pow(a, 2) + std::pow(offset - b, 2));
            }
            else {
                return a;
            }
        }

        if (angle >= DEG270INRAD and angle < DEG360INRAD) { // 270 - 360 deg
            angle = DEG360INRAD - angle;
            a = sin(angle) * distance;
            b = cos(angle) * distance;
            return std::sqrt(std::pow(a, 2) + std::pow(b + offset, 2));
        }
    }

/*    void EmergencyStop::projectOnRearAxle(float *angle, float *distance, double offset) {
        float tempAlpha = &angle;
        float tempDist = &distance;
        &angle = atan((sin(tempAlpha) * tempDist) / (cos(tempAlpha) * tempDist + offset));
        &distance = std::sqrt(std::pow(sin(tempAlpha) * tempDist, 2) + std::pow(cos(tempAlpha) * tempDist + offset, 2));
    }*/

    double EmergencyStop::getStraightDistanceToCar(double distanceToLidar, int deg_step) {
        if (deg_step < 90 || deg_step > 270) {
            return (distanceToLidar - config.forward_minimum_distance);
        } else {
            return (distanceToLidar - config.reverse_minimum_distance);
        }
    }

    double EmergencyStop::getExactDistanceToCar(double dist, double rad) {
        double offsetFront = config.forward_minimum_distance;
        double offsetRear = config.reverse_minimum_distance;
        double angleFront = config.half_angle_front_init;
        double angleRear = config.half_angle_back_init;
        double alpha;
        double a, b, c;

        // front: approx. same distance -> because of radial bumper
        if (angleFront >= rad && rad >= (DEG360INRAD - angleFront)) {
            return offsetFront;
        }
        if (angleFront < rad && rad <= DEG90INRAD) {
            alpha = DEG90INRAD - rad;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        if (DEG90INRAD < rad && rad < (DEG180INRAD - angleRear)) {
            alpha = rad - DEG90INRAD;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        if ((DEG180INRAD - angleRear) <= rad && rad < DEG180INRAD) {
            alpha = DEG180INRAD - rad;
            b = offsetRear;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        if (DEG180INRAD <= rad && rad <= (DEG180INRAD + offsetRear)) {
            alpha = rad - DEG180INRAD;
            b = offsetRear;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        if ((DEG180INRAD + offsetRear) < rad && rad <= DEG270INRAD) {
            alpha = DEG270INRAD - rad;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }
        if (DEG270INRAD < rad && rad < (DEG360INRAD - angleFront)) {
            alpha = rad - DEG270INRAD;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist - c;
        }

        // on coding error:
        return 0;
    }

    bool EmergencyStop::isOnCar(double dist, double rad) {
        double offsetFront = config.forward_minimum_distance;
        double offsetRear = config.reverse_minimum_distance;
        double angleFront = config.half_angle_front_init;
        double angleRear = config.half_angle_back_init;
        double alpha;
        double a, b, c;

        // front: approx. same distance -> because of radial bumper
        if (angleFront >= rad && rad >= (DEG360INRAD - angleFront)) {
            return dist < offsetFront;
        }
        if (angleFront < rad && rad <= DEG90INRAD) {
            alpha = DEG90INRAD - rad;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist < c;
        }
        if (DEG90INRAD < rad && rad < (DEG180INRAD - angleRear)) {
            alpha = rad - DEG90INRAD;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist < c;
        }
        if ((DEG180INRAD - angleRear) <= rad && rad < DEG180INRAD) {
            alpha = DEG180INRAD - rad;
            b = offsetRear;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist < c;
        }
        if (DEG180INRAD <= rad && rad <= (DEG180INRAD + offsetRear)) {
            alpha = rad - DEG180INRAD;
            b = offsetRear;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist < c;
        }
        if ((DEG180INRAD + offsetRear) < rad && rad <= DEG270INRAD) {
            alpha = DEG270INRAD - rad;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist < c;
        }
        if (DEG270INRAD < rad && rad < (DEG360INRAD - angleFront)) {
            alpha = rad - DEG270INRAD;
            b = config.car_width / 2;
            a = b * tan(alpha);
            c = hypotenuse(a, b);
            return dist < c;
        }

        // on coding error:
        return true;
    }

    double EmergencyStop::getTurningRadius(double steeringAngle) {
        return config.wheelbase / tan(steeringAngle);
    }

    double EmergencyStop::getX(double radius, double alpha, double d) {
        auto t = cos(alpha) * d;
        auto h = sin(alpha) * d;
        auto s = radius - t;
        auto tan_beta = h / s;
        auto beta = atan(tan_beta);
        auto x = s / cos(beta);
        return x;
    }

    bool EmergencyStop::isOnPath(double radius, double x) {
        return (x < (radius + config.car_width/2) && x > (radius - config.car_width/2));
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


    // works on actual lidar position & values
    bool EmergencyStop::isOnStraightPath(double posAngle, double distance, bool backward) {
        auto distanceFromMiddle = sin(posAngle) * distance;
        auto safetyWidth = 0.02;
        if(backward){
            safetyWidth = 0.04;
        }
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
        msg.data = static_cast<float>(currentTurningRadius);
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
            msg.value = wantedSpeed *
                        boost::algorithm::clamp(
                                obstacleDistance / (config.max_startup_damp_range * abs(wantedSpeed) / 1000), 0, 1); // dampen start up
            //boost::algorithm::clamp(obstacleDistance / (config.startup_damp_range + std::sqrt(wantedSpeed/1000)), 0, 1); // startup_damp_range = 1.0
            //std::pow(boost::algorithm::clamp(obstacleDistance / ((boost::algorithm::clamp(wantedSpeed, 200, 1000)/200) - 0.5), 0, 1), 2); // startup_damp_range = 1.0
        }
        return msg;
    }
}