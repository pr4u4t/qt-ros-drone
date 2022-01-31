#include "speedsubscriber.h"

/*------------------------------SPEED SUBSCRIBER------------------------------*/

SpeedSubscriber::SpeedSubscriber(QObject*){
    connect(this, SIGNAL(receivedSpeed(double)), this, SLOT(updateSpeed(double)));
    speed_sub = nh.subscribe("/odometry/filtered", 1, &SpeedSubscriber::speedCallback,this);
}

void SpeedSubscriber::speedCallback(const nav_msgs::Odometry::ConstPtr& msg){
    receivedSpeed(msg->twist.twist.linear.x);
}

void SpeedSubscriber::updateSpeed(double x){
    m_speed = x * MPS2KNOTS;
    emit speedChanged();
}
