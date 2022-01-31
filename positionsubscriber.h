#ifndef POSITIONSUBSCRIBER_H
#define POSITIONSUBSCRIBER_H

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

class PositionSubscriber : public QObject{
    Q_OBJECT
    Q_PROPERTY(float x MEMBER m_x NOTIFY positionXChnaged)
    Q_PROPERTY(float y MEMBER m_y NOTIFY positionYChanged)
    
    public:
        PositionSubscriber(QObject* parent = nullptr);
        virtual ~PositionSubscriber() {}

        void positionXCallback(const std_msgs::Float32::ConstPtr& msg);
	void positionYCallback(const std_msgs::Float32::ConstPtr& msg);

    private slots:
        void updatePosition(float x, float y);
	void updateXPosition(float x);
	void updateYposition(float y);

    signals:
        void positionChanged(float x = 0.0, float y = 0.0);
        void receivedPosition(float x = 0.0, float y = 0.0);
	void receivedXPosition(float x = 0.0);
	void receivedYPosition(float y = 0.0);
	void positionXChanged(float x = 0.0);
	void positionYChanged(float y = 0.0);

    private:
        float m_x = 0.0;
	float m_y = 0.0;

        ros::NodeHandle nh; 
        ros::Subscriber pos_sub_x;
	ros::Subscriber pos_sub_y;
};

#endif
