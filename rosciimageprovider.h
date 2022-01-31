#ifndef ROSCIIMAGEPROVIDER_H
#define ROSCIIMAGEPROVIDER_H

/*------------------------------COMPRESSED IMAGE SUBSCRIBER------------------------------*/
class RosCImageProvider : public QQuickImageProvider{
    public:
        RosCImageProvider();
        QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
        ImageType imageType() const override {return QQmlImageProviderBase::Image;}
    private:
        std::string _topic;
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        image_transport::ImageTransport _it;
        image_transport::Subscriber _sub;
        QImage _last_image;
};

#endif
