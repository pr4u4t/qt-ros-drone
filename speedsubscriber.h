#ifndef SPEED_SUBSCRIBER_H
#define SPEED_SUBSCRIBER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include <image_transport/image_transport.h>
#include "sensor_msgs/MagneticField.h"

/*------------------------------SPEED SUBSCRIBER------------------------------*/

class SpeedSubscriber : public QObject{
    Q_OBJECT
    Q_PROPERTY(double speed MEMBER m_speed NOTIFY speedChanged)

    public:
        SpeedSubscriber(QObject* parent = nullptr);
        virtual ~SpeedSubscriber() {}

        const double MPS2KNOTS = 1.94;

        void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);

    private slots:
        void updateSpeed(double x);

    signals:
        void speedChanged();
        void receivedSpeed(double x);

    private:
        double m_speed = 0.0;

        ros::NodeHandle nh;
        ros::Subscriber speed_sub;
};

#endif
