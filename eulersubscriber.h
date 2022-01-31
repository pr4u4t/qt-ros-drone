#ifndef EULER_SUBSCRIBER_H
#define EULER_SUBSCRIBER_H

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

/*------------------------------EULER SUBSCRIBER------------------------------*/

class EulerSubscriber : public QObject{
    Q_OBJECT
    Q_PROPERTY(double roll MEMBER m_roll NOTIFY rollChanged)
    Q_PROPERTY(double yaw MEMBER m_yaw NOTIFY yawChanged)
    Q_PROPERTY(double pitch MEMBER m_pitch NOTIFY pitchChanged)


    public:
        EulerSubscriber(QObject* parent = nullptr);
        virtual ~EulerSubscriber() {}



        void eulerCallback(const geometry_msgs::Vector3::ConstPtr& msg);
        const double RAD2DEG = 57.296;

    private slots:
        void updateroll(double x);
        void updateyaw(double y);
        void updatepitch(double z);


    signals:
        void rollChanged();
        void yawChanged();
        void pitchChanged();
        void receivedroll(double x);
        void receivedyaw(double y);
        void receivedpitch(double z);

    private:
        double m_roll = 0.0;
        double m_yaw = 0.0;
        double m_pitch = 0.0;
        double yaw = 0.0;

        ros::NodeHandle nh;
        ros::Subscriber euler_sub;
};

#endif
