#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig &config) { this->config = config; }

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {

        emergencyStop = false;

        double breakDistance = config.break_distance;
        double targetQuotient = config.target_quotient;
        double negAcc = config.negative_acceleration;
        double maxLookahead = config.maximum_lookahead_distance;

        auto minDistance = std::numeric_limits<double>::infinity(); // init minDist
        auto angleIncrement = scan->angle_increment;

        if (wantedSpeed >= 0) {    //forward.
            auto frontAngle = config.angle_front / 2.0;

            // front right
            auto start = 0;
            auto end = static_cast<int>(frontAngle / angleIncrement);
            for (int i = start; i < scan->ranges.size() && i < end; i++) {
                auto dist = getDistanceToCar(scan->ranges[i], i);
                if (dist < maxLookahead && dist > 0 && dist < minDistance)
                {
                    minDistance = dist;
                }
            }

            // front left
            start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
            end = scan->ranges.size();
            for (int k = start; k < end; k++) {
                auto dist = getDistanceToCar(scan->ranges[k], k);
                if (dist < maxLookahead && dist > 0 && dist < minDistance)
                {
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
                if (dist < maxLookahead && dist > 0 && dist < minDistance)
                {
                    minDistance = dist;
                }
            }
        }

        // based on speed, distance and deceleration, decide if EmergencyStop
        if (currentSpeed != 0) {
            if (safeDistanceQuotient(minDistance, negAcc, currentSpeed) <= targetQuotient ||
                minDistance <= breakDistance)
            {
                emergencyStop = true;
            }
        } else {
            if (minDistance <= breakDistance) {
                emergencyStop = true;
            }
        }

        /*
        if (currentSpeed==0){
            safeSpeed=std::sqrt((minDistance)/2.2);
            emergencyStop = true;
        }
        else if ((minDistance)/(2*std::pow(currentSpeed,2)) <= 1.1) {
            safeSpeed=std::sqrt((minDistance)/2.2);
            emergencyStop = true;
        }
         */

    }

    double EmergencyStop::getDistanceToCar(double distanceToLidar, int deg_step) {
        if(deg_step < 90 || deg_step > 270){
            return (distanceToLidar - config.forward_minimum_distance);
        }
        else{
            return (distanceToLidar - config.reverse_minimum_distance);
        }
    }

    double EmergencyStop::calculateSafeSpeed(double distance, double deacceleration, double targetQuotient) {
        return std::sqrt((minDistance) * deacceleration / targetQuotient);
    }

    double EmergencyStop::safeDistanceQuotient(double distance, double deacceleration, double currentSpeed) {
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

        /*
        if (emergencyStop && safeSpeed < wantedSpeed) {
            msg.value = safeSpeed;
        } else {
            msg.value = wantedSpeed;
        }
         */
        if (emergencyStop) {
            msg.value = 0;
        } else {
            msg.value = wantedSpeed;
        }

        return msg;
    }
}