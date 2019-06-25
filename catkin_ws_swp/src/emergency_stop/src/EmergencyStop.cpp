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

        if (currentSteeringAngle < 0.0) {
            orientation = Orientation::RIGHT;
        } else if (currentSteeringAngle > 0.0) {
            orientation = Orientation::LEFT;
        } else {
            orientation = Orientation::STRAIGHT;
        }

        double stopDistance = config.stop_distance;
        double targetQuotient = config.target_quotient;
        double negAcc = config.negative_acceleration;
        double maxLookahead = config.maximum_lookahead_distance;

        emergencyStop = false;
        auto minDistance = std::numeric_limits<double>::infinity(); // init minDist
        auto angleIncrement = scan->angle_increment;

        if (wantedSpeed >= 0) {    //forward
            auto frontAngle = config.angle_front / 2.0;

            // front right
            auto start = 0;
            auto end = static_cast<int>(frontAngle / angleIncrement);
            for (int i = start; i < scan->ranges.size() && i < end; i++) {
                auto dist = getDistanceToCar(scan->ranges[i], i);
                if (dist < maxLookahead && dist > 0 && dist < minDistance) {
                    minDistance = dist;
                }
            }

            // front left
            start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
            end = scan->ranges.size();
            for (int k = start; k < end; k++) {
                auto dist = getDistanceToCar(scan->ranges[k], k);
                if (dist < maxLookahead && dist > 0 && dist < minDistance) {
                    minDistance = dist;
                }
            }

        } else { //(wantedSpeed < 0) backward
            auto backAngle = config.angle_back / 2.0;
            int start = scan->ranges.size() / 2 - static_cast<int>(backAngle / angleIncrement);
            int end = scan->ranges.size() / 2 + static_cast<int>(backAngle / angleIncrement);
            // back right to back left
            for (int j = start; j < end && j < scan->ranges.size(); j++) {
                // we might see the camera in the laser scan
                auto dist = getDistanceToCar(scan->ranges[j], j);
                if (dist < maxLookahead && dist > 0 && dist < minDistance) {
                    minDistance = dist;
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

    double EmergencyStop::getDistanceToCar(double distanceToLidar, int deg_step) {
        /*
         * open for extension
         * --> give the exact distance for each orientation of the car to the obstacle
         */
        if (deg_step < 90 || deg_step > 270) {
            return (distanceToLidar - config.forward_minimum_distance);
        } else {
            return (distanceToLidar - config.reverse_minimum_distance);
        }
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

    bool EmergencyStop::evaluateMeasurePoint(double radius, double x) {
        return (x < (radius + config.car_width/2) && x > (radius - config.car_width/2));
    }

    double EmergencyStop::calculateSafeSpeed(double distance, double deceleration, double targetQuotient) {
        return std::sqrt((distance) * deceleration / targetQuotient);
    }

    double EmergencyStop::safeDistanceQuotient(double distance, double deceleration, double currentSpeed) {
        if (currentSpeed == 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return distance * deceleration / std::pow(currentSpeed, 2);
    }

    void EmergencyStop::setCurrentSpeed(const autominy_msgs::SpeedConstPtr &speed) {
        currentSpeed = speed->value;
    }

    void EmergencyStop::setCurrentSteeringAngle(const autominy_msgs::SteeringAngleConstPtr &steering) {
        currentSteeringAngle = steering->value;
    }

    void EmergencyStop::setWantedSpeed(const autominy_msgs::SpeedCommandConstPtr &speed) {
        wantedSpeed = speed->value;
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
                                obstacleDistance / (config.max_startup_damp_range * abs(wantedSpeed) / 1000), 0,
                                1); // dampen start up
            //boost::algorithm::clamp(obstacleDistance / (config.startup_damp_range + std::sqrt(wantedSpeed/1000)), 0, 1); // startup_damp_range = 1.0
            //std::pow(boost::algorithm::clamp(obstacleDistance / ((boost::algorithm::clamp(wantedSpeed, 200, 1000)/200) - 0.5), 0, 1), 2); // startup_damp_range = 1.0
        }
        return msg;
    }
}