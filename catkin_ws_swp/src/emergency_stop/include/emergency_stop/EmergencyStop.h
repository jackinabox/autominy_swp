#pragma once

#include <emergency_stop/EmergencyStopConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SpeedCommand.h>
#include <autominy_msgs/SteeringAngle.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <limits>
#include <math.h>
#include <string>
#include <vector>
#include <boost/algorithm/clamp.hpp>

namespace emergency_stop {

    const double DEG90INRAD =  M_PI * 0.5; // 1.5707963267948966;
    const double DEG180INRAD = M_PI;       // 3.1415926535897931;
    const double DEG270INRAD = M_PI * 1.5; // 4.7123889803846897;
    const double DEG360INRAD = M_PI * 2;   // 6.2831853071795862;

    enum class Direction : int8_t {
        FORWARD = 1,
        BACKWARD = -1
    };

    enum class Steering : int8_t {
        STRAIGHT = 0,
        LEFT = 1,
        RIGHT = -1
    };

    /** EmergencyStop class. Contains the general functionality of this package.
     **
     ** @ingroup @@
     */

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

        std_msgs::Float32 getDistanceFromCar();

        std_msgs::Float32 getSteeringAngleSub();

        std_msgs::Float32 getCurrentTurningRadius();


    private:
        /// dynamic config attribute
        emergency_stop::EmergencyStopConfig config;
        double turningRadius;
        double turningRadiusIR;
        double turningRadiusOF;
        double currentSteeringAngle = 0.0;
        double obstacleDistance;
        double obstacleDistanceActual;
        double currentSpeed = 0.0;
        double safeSpeedSI = 0.0;
        int16_t safeSpeedPWM = 0;
        int16_t wantedSpeed = 0;
        bool emergencyStop = true;

        double hypotenuse(double a, double b);

        double getDistanceToCarOnPath(double angle, double distance, double turningRadius, double turningRadiusIR, double turningRadiusOF, Direction direction, Steering steering);

        double getStraightDistanceToCar(double distanceToLidar, int deg_step);

        double getExactDistanceToCar(double rad, double dist);

        double getTurningRadius(double steeringAngle);

        double getAlignedAngle(double angle, double coordRot = DEG90INRAD);

        void alignCoordToFront(double &angle, double coordRot = DEG90INRAD);

        std::tuple<double, double> polar2Cart(double r, double theta);

        std::tuple<double, double> cart2Polar(double x, double y);

        double getEuclideanDistance(double x_center, double y_center, double x_obstacle, double y_obstacle);

        double getAngleBetweenVectors(std::vector<double> a, std::vector<double> b);

        double calcOffsetAngleFront(double distance_from_center, Direction direction);

        double calcOffsetAngleSide(double distance_from_center, double r_ir);

        double calcOffsetAngle(double distance_from_center, double r_ir, Direction direction);

        double calcEffectiveDistance(double angle, double angleOffset, double radius);

        double calculateSafeSpeed(double distance, double deceleration, double targetQuotient);

        double safeDistanceQuotient(double distance, double deceleration, double currentSpeed);

        bool isOnStraightPath(double posAngle, double distance, bool backward);

        bool isOnPath(double x, double rIR, double rOF);

        bool isOnCar(double rad, double dist);

        std::string steers = "straight";

        Direction direction = Direction::FORWARD;

        Steering steering = Steering::STRAIGHT;

    };
}
