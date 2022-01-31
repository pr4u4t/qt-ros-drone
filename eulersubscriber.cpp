#include "eulersubscriber.h"

/*------------------------------EULER SUBSCRIBER------------------------------*/

EulerSubscriber::EulerSubscriber(QObject*){
    connect(this, SIGNAL(receivedroll(double)), this, SLOT(updateroll(double)));
    connect(this, SIGNAL(receivedyaw(double)), this, SLOT(updateyaw(double)));
    connect(this, SIGNAL(receivedpitch(double)),this, SLOT(updatepitch(double)));
    euler_sub = nh.subscribe("/euler", 1, &EulerSubscriber::eulerCallback,this);
}

void EulerSubscriber::eulerCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    //yaw = atan2(msg->magnetic_field.x, msg->magnetic_field.y);
    receivedpitch(msg->x);//pitch
    receivedyaw(msg->z);//yaw
    receivedroll(msg->y);//roll
}

void EulerSubscriber::updateroll(double x){
    m_roll = x * RAD2DEG;
    emit rollChanged();
}

void EulerSubscriber::updateyaw(double y){
    m_yaw = y * RAD2DEG;
    emit yawChanged();
    //qDebug(m_yaw);
}

void EulerSubscriber::updatepitch(double z){
    m_pitch = z * RAD2DEG;
    emit pitchChanged();
}
