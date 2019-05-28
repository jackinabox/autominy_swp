#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig &config) { this->config = config; }

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {
        double breakDistance = config.break_distance;
        emergencyStop = false;

        auto angleIncrement = scan->angle_increment;
        if (wantedSpeed >= 0) {    //forward.
            auto frontAngle = config.angle_front / 2.0;

            // front right
            auto start = 0;
            auto end = static_cast<int>(frontAngle / angleIncrement);

            auto minDistance=scan->ranges[0]-config.forward_minimum_distance; // init minDist forward
            for (int i = 1; i < scan->ranges.size() && i < end; i++) {
                if ((scan->ranges[i]-config.forward_minimum_distance)<minDistance && scan->ranges[i]>config.forward_minimum_distance){
                    minDistance=scan->ranges[i]-config.forward_minimum_distance;
                }
            }

            // front left
            start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
            end = scan->ranges.size();
            for (int k = start; k < end; k++) {
                if ((scan->ranges[k]-config.forward_minimum_distance)<minDistance && scan->ranges[k]>config.forward_minimum_distance){
                    minDistance=scan->ranges[k]-config.forward_minimum_distance;
                }
            }

            if (currentSpeed!=0){
                if((minDistance)/(2*std::pow(currentSpeed, 2)) <= 1.1 || minDistance <= breakDistance){
                    emergencyStop = true;
                }
            }
            else{
                if (minDistance <= breakDistance){
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

        else{ //(wantedSpeed < 0) backward.
            auto backAngle = config.angle_back / 2.0;
            int start = scan->ranges.size() / 2 - static_cast<int>(backAngle / angleIncrement);
            int end = scan->ranges.size() / 2 + static_cast<int>(backAngle / angleIncrement);
            // back right to back left
            auto minDistance=scan->ranges[start]-config.reverse_minimum_distance; // init backward minDist
            for (int j = start; j < end && j < scan->ranges.size(); j++) {
                // we might see the camera in the laser scan
                if ((scan->ranges[j]-config.reverse_minimum_distance)<minDistance && scan->ranges[j] > config.reverse_minimum_distance){
                    minDistance=scan->ranges[j]-config.reverse_minimum_distance;
                }
            }

            if (currentSpeed!=0){
                if((minDistance)/(2*std::pow(currentSpeed, 2)) <= 1.1 || minDistance <= breakDistance){
                    emergencyStop = true;
                }
            }
            else{
                if (minDistance <= breakDistance){
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
    }

    double EmergencyStop::calculateSafeSpeed(double distance,double deacceleration, double targetQuotient){
        return std::sqrt((minDistance)*deacceleration/targetQuotient)
    }

    double EmergencyStop::safeDistanceQuotient(double distance,double deacceleration, double currentSpeed){
        return distance*deacceleration/std::pow(currentSpeed,2)
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