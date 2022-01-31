#include "positionsubscriber.h"

/*------------------------------POSITION SUBSCRIBER------------------------------*/

PositionSubscriber::PositionSubscriber(QObject* parent){
    connect(this, SIGNAL(receivedXPosition(float)), this, SLOT(updateXPosition(float)));
    connect(this, SIGNAL(receivedYPosition(float)), this, SLOT(updateYPosition(float)));

    pos_sub_x = nh.subscribe("/xpos", 1, &PositionSubscriber::positionXCallback, this);
    pos_sub_y = nh.subscribe("/ypos", 1, &PositionSubscriber::positionYCallback, this);
}

void PositionSubscriber::positionXCallback(const std_msgs::Float32::ConstPtr& msg){
    emit receivedXPosition(msg->data);
}

void PositionSubscriber::positionYCallback(const std_msgs::Float32::ConstPtr& msg){
    emit receivedYPosition(msg->data);
}

void PositionSubscriber::updatePosition(float x, float y){
	m_x = x;
	m_y = y;
	emit positionChanged(x,y);
} 

void PositionSubscriber::updateXPosition(float x){
	m_x = x;
	emit positionXChanged(x);
}
        
void PositionSubscriber::updateYPosition(float y){
	m_y = y;
	emit positionYChanged(y);
}

