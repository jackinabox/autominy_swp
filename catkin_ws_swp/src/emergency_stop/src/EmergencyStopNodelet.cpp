#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <emergency_stop/EmergencyStopFwd.h>
#include <emergency_stop/EmergencyStopConfig.h>
#include <emergency_stop/EmergencyStop.h>
#include <sensor_msgs/LaserScan.h>
#include <autominy_msgs/SpeedCommand.h>
#include <autominy_msgs/SteeringAngle.h>
#include <autominy_msgs/Speed.h>



namespace emergency_stop {

/** EmergencyStop nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class EmergencyStopNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        EmergencyStopNodelet() = default;

        /** Destructor.
         */
        ~EmergencyStopNodelet() override {}

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        virtual void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            emergencyStop = std::make_shared<EmergencyStop>();

            distanceToObstaclePublisher = pnh.advertise<std_msgs::Float32>("obstacle_distance", 1);
            directDistanceToObstaclePublisher = pnh.advertise<std_msgs::Float32>("obstacle_distance_actual", 1);
            speedPublisher = pnh.advertise<autominy_msgs::SpeedCommand>("speed", 1);
            safeSpeedPublisher = pnh.advertise<autominy_msgs::SpeedCommand>("safeSpeed", 1);
            steerPublisher = pnh.advertise<std_msgs::String>("car_steers", 1);
            steerAngleSubPublisher = pnh.advertise<std_msgs::Float32>("car_steers_angle", 1);
            turningRadiusPublisher = pnh.advertise<std_msgs::Float32>("turning_radius", 1);


            scanSubscriber = pnh.subscribe("scan", 1, &EmergencyStopNodelet::onScan, this, ros::TransportHints().tcpNoDelay());
            wantedSpeedSubscriber = pnh.subscribe("wanted_speed", 1, &EmergencyStopNodelet::onWantedSpeed, this, ros::TransportHints().tcpNoDelay());
            currentSpeedSubscriber = pnh.subscribe("carstate/speed", 1, &EmergencyStopNodelet::onCurrentSpeed, this, ros::TransportHints().tcpNoDelay());
            currentSteeringAngleSubscriber = pnh.subscribe("/carstate/steering", 1, &EmergencyStopNodelet::onCurrentSteeringAngle, this, ros::TransportHints().tcpNoDelay());

            config_server_ = boost::make_shared<dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig> >(pnh);
            dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig>::CallbackType f;
            f = boost::bind(&EmergencyStopNodelet::callbackReconfigure, this, _1, _2);
            config_server_->setCallback(f);
        }

    private:
        /** Callback for messages of some type.
         **
         ** @param msg
         */
        void onScan(sensor_msgs::LaserScanConstPtr const &msg) {
            emergencyStop->checkEmergencyStop(msg);
            speedPublisher.publish(emergencyStop->getSpeedToPublish());
            distanceToObstaclePublisher.publish(emergencyStop->getDistanceToObstacle());
            directDistanceToObstaclePublisher.publish(emergencyStop->getDistanceFromCar());
            safeSpeedPublisher.publish(emergencyStop->getSafeSpeed());
            steerPublisher.publish(emergencyStop->getSteering());
            steerAngleSubPublisher.publish(emergencyStop->getSteeringAngleSub());
            turningRadiusPublisher.publish(emergencyStop->getCurrentTurningRadius());
        }

        void onCurrentSpeed(autominy_msgs::SpeedConstPtr const &msg) {
            emergencyStop->setCurrentSpeed(msg);
        }

        void onWantedSpeed(autominy_msgs::SpeedCommandConstPtr const &msg) {
            emergencyStop->setWantedSpeed(msg);
        }

        void onCurrentSteeringAngle(autominy_msgs::SteeringAngleConstPtr const &msg) {
            emergencyStop->setCurrentSteeringAngle(msg);
        }

        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(emergency_stop::EmergencyStopConfig &config, uint32_t level) {
            emergencyStop->setConfig(config);
        }

        /// subscriber
        ros::Subscriber scanSubscriber;
        ros::Subscriber currentSpeedSubscriber;
        ros::Subscriber wantedSpeedSubscriber;
        ros::Subscriber currentSteeringAngleSubscriber;

        /// publisher
        ros::Publisher speedPublisher;
        ros::Publisher safeSpeedPublisher;
        ros::Publisher distanceToObstaclePublisher;
        ros::Publisher directDistanceToObstaclePublisher;
        ros::Publisher steerPublisher;
        ros::Publisher steerAngleSubPublisher;
        ros::Publisher turningRadiusPublisher;


        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig> > config_server_;

        /// pointer to the functionality class
        EmergencyStopPtr emergencyStop;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(emergency_stop::EmergencyStopNodelet, nodelet::Nodelet);
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
