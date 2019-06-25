#pragma once

#include <emergency_stop/EmergencyStopConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SpeedCommand.h>
#include <autominy_msgs/SteeringAngle.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <limits>
#include <math.h>
#include <string>
#include <boost/algorithm/clamp.hpp>

namespace emergency_stop {

/** EmergencyStop class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */

    enum class Direction : int8_t {
        FORWARD = 1,
        BACKWARD = -1
    };

    enum class Orientation : int8_t {
        STRAIGHT = 0,
        LEFT = 1,
        RIGHT = -1
    };

    class EmergencyStop {
    public:
        /** Constructor.
         */
        EmergencyStop();

        /** Destructor.
         */
        virtual ~EmergencyStop();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(emergency_stop::EmergencyStopConfig &config);

        void checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan);

        void setCurrentSpeed(const autominy_msgs::SpeedConstPtr &speed);

        void setWantedSpeed(const autominy_msgs::SpeedCommandConstPtr &speed);

        void setCurrentSteeringAngle(const autominy_msgs::SteeringAngleConstPtr &steering);

        std_msgs::String getSteering();

        autominy_msgs::SpeedCommand getSafeSpeed();

        autominy_msgs::SpeedCommand getSpeedToPublish();

        std_msgs::Float32 getDistanceToObstacle();

        std_msgs::Float32 getSteeringAngleSub();

    private:
        /// dynamic config attribute
        emergency_stop::EmergencyStopConfig config;
        double currentSteeringAngle;
        double obstacleDistance;
        double currentSpeed = 0.0;
        double safeSpeedSI = 0.0;
        int16_t safeSpeedPWM = 0;
        int16_t wantedSpeed = 0;
        bool emergencyStop = true;

        double getDistanceToCar(double distanceToLidar, int deg_step);

        double getTurningRadius(double steeringAngle);

        double getX(double radius, double alpha, double d);

        bool evaluateMeasurePoint(double radius, double x);

        double calculateSafeSpeed(double distance, double deacceleration, double targetQuotient);

        double safeDistanceQuotient(double distance, double deacceleration, double currentSpeed);

        std::string steers = "straight";

        Direction direction = Direction::FORWARD;

        Orientation orientation = Orientation ::STRAIGHT;
    };
}
