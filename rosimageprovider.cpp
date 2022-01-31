#include "rosimageprovider.h"

/*------------------------------IMAGE SUBSCRIBER------------------------------*/
RosImageProvider::RosImageProvider() : QQuickImageProvider((QQuickImageProvider::Pixmap)), _it(ros::NodeHandle()), _last_image(QImage(10,10, QImage::Format_RGB888)){
    _last_image.fill(QColor("black").rgba());
}

void RosImageProvider::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    _last_image = QImage(msg->width, msg->height, QImage::Format_RGB888);
    memcpy(_last_image.bits(), msg->data.data(), _last_image.sizeInBytes());
}

QImage RosImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize){
    if (_topic != id.toStdString()) {
        _topic = id.toStdString();
        _sub = _it.subscribe(_topic, 1, &RosImageProvider::imageCallback, this);
    }
    QImage result;

    if (requestedSize.isValid()){
        result = _last_image.scaled(requestedSize, Qt::KeepAspectRatio);

    }
    else {
        result = _last_image;
    }
    
    *size = result.size();
    return result;
}
