#include <QObject>

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

class RpmSubscriber : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double rpmdata MEMBER m_rpm NOTIFY rpmChanged)
    Q_PROPERTY(double currentdata MEMBER m_current NOTIFY currentChanged)
    Q_PROPERTY(double voltagedata MEMBER m_voltage NOTIFY voltageChanged)
    Q_PROPERTY(double throttledata MEMBER m_throttle NOTIFY throttleChanged)
    Q_PROPERTY(double pressuredata MEMBER m_pressure NOTIFY pressureChanged)
    Q_PROPERTY(double depthdata MEMBER m_depth NOTIFY depthChanged)
    Q_PROPERTY(double zpathdata MEMBER m_zpath NOTIFY zPathChanged)
    Q_PROPERTY(double inflowdata MEMBER m_inflow NOTIFY inflowChanged)
    Q_PROPERTY(double outflowdata MEMBER m_outflow NOTIFY outflowChanged)
    Q_PROPERTY(double inletpumpdata MEMBER m_inletpump NOTIFY inletpumpChanged)
    Q_PROPERTY(double outletpumpdata MEMBER m_outletpump NOTIFY outletpumpChanged)

public:
    RpmSubscriber(QObject* parent = nullptr);
    virtual ~RpmSubscriber() {}

    const double RAD2RPM = -9.55;
    const double zero_rpm = 0;
    int rpm_count =0;
    const double AMPS = 0.001;
    const double VOLTS = 0.001;
    //const double GAIN = 0.0002;

    void rpmCallback(const std_msgs::UInt32::ConstPtr& msg);
    void currentCallback(const std_msgs::Float32::ConstPtr& msg);
    void voltageCallback(const std_msgs::Float32::ConstPtr& msg);
    void throttleCallback(const std_msgs::Int16::ConstPtr& msg);
    void pressureCallback(const std_msgs::Float32::ConstPtr& msg);
    void depthCallback(const std_msgs::Float32::ConstPtr& msg);
    void zpathCallback(const std_msgs::Float32::ConstPtr& msg);
    void inflowCallback(const std_msgs::Float32::ConstPtr& msg);
    void outflowCallback(const std_msgs::Float32::ConstPtr& msg);
    void inletpumpCallback(const std_msgs::Int32::ConstPtr& msg);
    void outletpumpCallback(const std_msgs::Int32::ConstPtr& msg);

private slots:
    void updaterpm(double data);
    void updatecurrent(double data);
    void updatevoltage(double data);
    void updatethrottle(double data);
    void updatepressure(double data);
    void updatedepth(double data);
    void updatezpath(double data);
    void updateinflow(double data);
    void updateoutflow(double data);
    void updateinletpump(double data);
    void updateoutletpump(double data);

signals:
    void rpmChanged();
    void currentChanged();
    void voltageChanged();
    void throttleChanged();
    void pressureChanged();
    void depthChanged();
    void zPathChanged();
    void inflowChanged();
    void outflowChanged();
    void inletpumpChanged();
    void outletpumpChanged();
    void receivedrpm(double data);
    void receivedcurrent(double data);
    void receivedvoltage(double data);
    void receivedthrottle(double data);
    void receivedpressure(double data);
    void receiveddepth(double data);
    void receivedzpath(double data);
    void receivedinflow(double data);
    void receivedoutflow(double data);
    void receivedinletpump(double data);
    void receivedoutletpump(double data);

private:
    double m_rpm = 0.0;
    double prev_rpm = 0.0;
    double m_current = 0.0;
    double m_voltage = 0.0;
    double m_throttle = 0.0;
    double m_pressure = 0.0;
    double m_depth = 0.0;
    double m_zpath = 0.0;
    double m_inflow = 0.0;
    double m_outflow = 0.0;
    double m_inletpump = 0.0;
    double m_outletpump = 0.0;

    ros::NodeHandle nh;
    ros::Subscriber rpm_sub;
    ros::Subscriber current_sub;
    ros::Subscriber voltage_sub;
    ros::Subscriber throttle_sub;
    ros::Subscriber pressure_sub;
    ros::Subscriber depth;
    ros::Subscriber zpath;
    ros::Subscriber inflow_sub;
    ros::Subscriber outflow_sub;
    ros::Subscriber inletpump_sub;
    ros::Subscriber outletpump_sub;
};
