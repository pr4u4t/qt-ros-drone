#include "batterysubscriber.h"

/*------------------------------BATTERY SUBSCRIBER------------------------------*/

class BatterySubscriber : public QObject{
    Q_OBJECT
    Q_PROPERTY(double battery MEMBER m_battery NOTIFY batteryChanged)

    public:
        BatterySubscriber(QObject* parent = nullptr);
        virtual ~BatterySubscriber() {}

        //const double MPS2KNOTS = 1.94;

        void batteryCallback(const std_msgs::Float32::ConstPtr& msg);

    private slots:
        void updateBattery(double x);

    signals:
        void batteryChanged();
        void receivedBattery(double x);

    private:
        double m_battery = 0.0;

        ros::NodeHandle nh;
        ros::Subscriber battery_sub;
};
