#include "main.h"

using namespace std;

int main(int argc, char** argv)
{
    QString ip;
    QByteArray rosaddress;
    QString jsstring;
    std::string jsport;
    QFile f("test.txt");
    QFile j("joystick.txt");


    QTextStream in(&f);
    if(f.open(QIODevice::ReadWrite)){
       ip = in.readAll();
       rosaddress.append(ip);
    }
    QTextStream inj(&j);
    if(j.open(QIODevice::ReadWrite)){
               jsstring = inj.readLine();
               jsport = jsstring.toStdString();

    }
    qDebug("Value is");
    qDebug(rosaddress);
    qDebug(jsstring.toLatin1());

    qputenv("ROS_MASTER_URI", rosaddress);


    // Init ROS
    ros::init(argc, argv, "UI");
    ros::NodeHandle node;

    // Init Qt
    QGuiApplication app(argc, argv);
    qmlRegisterType<Graph>("graph",1,0,"Graph");

    // Subscribers and Publishers
    SpeedSubscriber speedSubscriber(&app);
    EulerSubscriber eulerSubscriber(&app);
    RpmSubscriber rpmSubscriber(&app);
    BatterySubscriber batterySubscriber(&app);
    PositionSubscriber positionSubscriber(&app);

    // Start ROS in separate thread, and trigger Qt shutdown when it exits
    // If Qt exits before ROS, be sure to shutdown ROS
    QFutureWatcher<void> rosThread;
    rosThread.setFuture(QtConcurrent::run(&ros::spin));
    QObject::connect(&rosThread, &QFutureWatcher<void>::finished, &app, &QCoreApplication::quit);
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [](){ros::shutdown();});

    QQmlApplicationEngine engine(&app);

    engine.rootContext()->setContextProperty("x", &positionSubscriber);
    engine.rootContext()->setContextProperty("y", &positionSubscriber);
    engine.rootContext()->setContextProperty("speed", &speedSubscriber);
    engine.rootContext()->setContextProperty("roll", &eulerSubscriber);
    engine.rootContext()->setContextProperty("yaw", &eulerSubscriber);
    engine.rootContext()->setContextProperty("pitch", &eulerSubscriber);
    engine.rootContext()->setContextProperty("rpmdata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("currentdata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("voltagedata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("throttledata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("depthdata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("zpathdata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("pressuredata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("battery", &batterySubscriber);
    engine.rootContext()->setContextProperty("inflowdata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("outflowdata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("inletpumpdata", &rpmSubscriber);
    engine.rootContext()->setContextProperty("outletpumpdata", &rpmSubscriber);
    engine.addImageProvider("rosimage", new RosImageProvider);
    engine.addImageProvider("roscimage", new RosCImageProvider);

    engine.addImportPath("qrc:/");
    engine.load(QUrl("qrc:/main.qml"));
    std::thread joy(jsqt, jsport);

    // Start Qt app
    return app.exec();
}
