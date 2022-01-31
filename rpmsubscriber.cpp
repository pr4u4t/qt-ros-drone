/*------------------------------RPM, CURRENT, VOLTAGE SUBSCRIBER------------------------------*/

RpmSubscriber::RpmSubscriber(QObject*)
{
    connect(this, SIGNAL(receivedrpm(double)), this, SLOT(updaterpm(double)));
    connect(this, SIGNAL(receivedcurrent(double)), this, SLOT(updatecurrent(double)));
    connect(this, SIGNAL(receivedvoltage(double)), this, SLOT(updatevoltage(double)));
    connect(this, SIGNAL(receivedthrottle(double)), this, SLOT(updatethrottle(double)));
    connect(this, SIGNAL(receivedpressure(double)), this, SLOT(updatepressure(double)));
    connect(this, SIGNAL(receiveddepth(double)), this, SLOT(updatedepth(double)));
    connect(this, SIGNAL(receivedzpath(double)), this, SLOT(updatezpath(double)));
    connect(this, SIGNAL(receivedinflow(double)), this, SLOT(updateinflow(double)));
    connect(this, SIGNAL(receivedoutflow(double)), this, SLOT(updateoutflow(double)));
    connect(this, SIGNAL(receivedinletpump(double)), this, SLOT(updateinletpump(double)));
    connect(this, SIGNAL(receivedoutletpump(double)), this, SLOT(updateoutletpump(double)));
    rpm_sub = nh.subscribe("/rpm_bldc", 1, &RpmSubscriber::rpmCallback,this);
    current_sub = nh.subscribe("/current", 1, &RpmSubscriber::currentCallback,this);
    voltage_sub = nh.subscribe("/voltage", 1, &RpmSubscriber::voltageCallback,this);
    throttle_sub = nh.subscribe("/throttle", 1, &RpmSubscriber::throttleCallback,this);
    pressure_sub = nh.subscribe("/pressure_ballast", 1, &RpmSubscriber::pressureCallback,this);
    depth = nh.subscribe("/depth", 1, &RpmSubscriber::depthCallback,this);
    zpath = nh.subscribe("/zpos", 1, &RpmSubscriber::zpathCallback,this);
    inflow_sub = nh.subscribe("/flow_in", 1, &RpmSubscriber::inflowCallback,this);
    outflow_sub = nh.subscribe("/flow_out", 1, &RpmSubscriber::outflowCallback,this);
    inletpump_sub = nh.subscribe("/inlet_pump", 1, &RpmSubscriber::inletpumpCallback,this);
    outletpump_sub = nh.subscribe("/outlet_pump", 1, &RpmSubscriber::outletpumpCallback,this);
}

void RpmSubscriber::rpmCallback(const std_msgs::UInt32::ConstPtr& msg)
{
    if (msg->data <= 100 or msg->data >=4500){
        if (rpm_count>2){
            receivedrpm(zero_rpm);
            prev_rpm = zero_rpm;
            rpm_count = 0;
        }
        else{
            receivedrpm(prev_rpm);
            rpm_count= rpm_count + 1;
            
            //qDebug(rpm_count);
        }
    }
    
    else {
        receivedrpm(msg->data);
        prev_rpm = msg->data;
        rpm_count = 0;
    }
    
    //receivedcurrent(msg->data);
    //receivedvoltage(msg->data);
    //receivedthrottle(msg->data);
}
void RpmSubscriber::currentCallback(const std_msgs::Float32::ConstPtr &msg){
    receivedcurrent(msg->data);
}

void RpmSubscriber::voltageCallback(const std_msgs::Float32::ConstPtr &msg){
    receivedvoltage(msg->data);
}

void RpmSubscriber::throttleCallback(const std_msgs::Int16::ConstPtr &msg){
    receivedthrottle(msg->data);
}

void RpmSubscriber::pressureCallback(const std_msgs::Float32::ConstPtr &msg){
    receivedpressure(msg->data);
}

void RpmSubscriber::depthCallback(const std_msgs::Float32::ConstPtr &msg){
    receiveddepth(msg->data);
}

void RpmSubscriber::zpathCallback(const std_msgs::Float32::ConstPtr &msg){
    receivedzpath(msg->data);
}

void RpmSubscriber::inflowCallback(const std_msgs::Float32::ConstPtr &msg){
    receivedinflow(msg->data);
}

void RpmSubscriber::outflowCallback(const std_msgs::Float32::ConstPtr &msg){
    receivedoutflow(msg->data);
}

void RpmSubscriber::inletpumpCallback(const std_msgs::Int32::ConstPtr &msg){
    receivedinletpump(msg->data);
}

void RpmSubscriber::outletpumpCallback(const std_msgs::Int32::ConstPtr &msg){
    receivedoutletpump(msg->data);
}

void RpmSubscriber::updaterpm(double data){
    m_rpm = data  ;
    emit rpmChanged();
}

void RpmSubscriber::updatecurrent(double data){
    m_current = data * AMPS;
    emit currentChanged();
}

void RpmSubscriber::updatevoltage(double data){
    m_voltage = data * VOLTS;
    emit voltageChanged();
}

void RpmSubscriber::updatethrottle(double data){
    m_throttle = data;
    emit throttleChanged();
}

void RpmSubscriber::updatepressure(double data){
    m_pressure = data;
    emit pressureChanged();
}

void RpmSubscriber::updatedepth(double data){
    m_depth = data;
    emit depthChanged();
}

void RpmSubscriber::updatezpath(double data){
    m_zpath = data;
    emit zPathChanged();
}

void RpmSubscriber::updateinflow(double data){
    m_inflow = data;
    emit inflowChanged();
}

void RpmSubscriber::updateoutflow(double data){
    m_outflow = data;
    emit outflowChanged();
}

void RpmSubscriber::updateinletpump(double data){
    m_inletpump = data;
    emit inletpumpChanged();
}

void RpmSubscriber::updateoutletpump(double data){
    m_outletpump = data;
    emit outletpumpChanged();
}
