#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig &config) { this->config = config; }

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {

        if (emergencyStop == false) {
            safeSpeedPWM = wantedSpeed;
        }

        double breakDistance = config.break_distance;
        double targetQuotient = config.target_quotient;
        double negAcc = config.negative_acceleration;
        double maxLookahead = config.maximum_lookahead_distance;

        emergencyStop = false;
        auto minDistance = std::numeric_limits<double>::infinity(); // init minDist
        auto angleIncrement = scan->angle_increment;

        if (wantedSpeed >= 0) {    //forward.
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

        } else { //(wantedSpeed < 0) backward.
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

        // based on speed, distance and deceleration, decide if EmergencyStop
        if (minDistance <= breakDistance) {
            safeSpeedPWM = 0;
            /*
            autominy_msgs::SpeedCommand msg;
            msg.value = safeSpeedPWM;
            safeSpeedPublisher.publish(msg);
            */
            emergencyStop = true;
            return;
        }
        if (safeDistanceQuotient(minDistance, negAcc, currentSpeed) <= targetQuotient) {
            safeSpeedSI = calculateSafeSpeed(minDistance, negAcc, targetQuotient);
            auto speedQuotient = safeSpeedSI / currentSpeed;
            safeSpeedPWM = static_cast<int16_t>(safeSpeedPWM * speedQuotient);
            //cout << safeSpeedPWM;
            /*
            autominy_msgs::SpeedCommand msg;
            msg.value = safeSpeedPWM;
            safeSpeedPublisher.publish(msg);
            */
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

    double EmergencyStop::calculateSafeSpeed(double distance, double deacceleration, double targetQuotient) {
        return std::sqrt((distance) * deacceleration / targetQuotient);
    }

    double EmergencyStop::safeDistanceQuotient(double distance, double deacceleration, double currentSpeed) {
        if (currentSpeed == 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return distance * deacceleration / std::pow(currentSpeed, 2);
    }

    void EmergencyStop::setCurrentSpeed(const autominy_msgs::SpeedConstPtr &speed) {
        currentSpeed = speed->value;
    }

    void EmergencyStop::setWantedSpeed(const autominy_msgs::SpeedCommandConstPtr &speed) {
        wantedSpeed = speed->value;
    }

    autominy_msgs::SpeedCommand EmergencyStop::getSafeSpeed() {
        autominy_msgs::SpeedCommand msg;
        if (emergencyStop && safeSpeedPWM < wantedSpeed) {
            msg.value = safeSpeedPWM;
        } else {
            msg.value = wantedSpeed;
        }
        return msg;
    }
}