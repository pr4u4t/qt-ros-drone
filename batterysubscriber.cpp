#include "batterysubscriber.h"

/*------------------------------BATTERY SUBSCRIBER------------------------------*/

BatterySubscriber::BatterySubscriber(QObject*){
    connect(this, SIGNAL(receivedBattery(double)), this, SLOT(updateBattery(double)));
    battery_sub = nh.subscribe("/battery", 1, &BatterySubscriber::batteryCallback,this);
}

void BatterySubscriber::batteryCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if (((msg->data)/4)>4.2 or ((msg->data)/4)==4.2){
        receivedBattery(100);
    }
    else if (((msg->data)/4)>4.1 or ((msg->data)/4)==4.1){
        receivedBattery(90);
    }
    else if (((msg->data)/4)>4.0 or ((msg->data)/4)==4.0){
        receivedBattery(80);
    }
    else if (((msg->data)/4)>3.9 or ((msg->data)/4)==3.9){
        receivedBattery(70);
    }
    else if (((msg->data)/4)>3.8 or ((msg->data)/4)==3.8){
        receivedBattery(60);
    }
    else if (((msg->data)/4)>3.7 or ((msg->data)/4)==3.7){
        receivedBattery(50);
    }
    else if (((msg->data)/4)>3.6 or ((msg->data)/4)==3.6){
        receivedBattery(40);
    }
    else if (((msg->data)/4)>3.5 or ((msg->data)/4)==3.5){
        receivedBattery(30);
    }
    else if (((msg->data)/4)>3.4 or ((msg->data)/4)==3.4){
        receivedBattery(20);
    }
    else if (((msg->data)/4)>3.3 or ((msg->data)/4)==3.3){
        receivedBattery(10);
    }
    else if (((msg->data)/4)>3.2 or ((msg->data)/4)==3.2){
        receivedBattery(0);
    }
    else if (((msg->data)/4)<3.1 or ((msg->data)/4)==3.1){
        receivedBattery(0);
    }
}

void BatterySubscriber::updateBattery(double x){
    m_battery = x;
    emit batteryChanged();
}
