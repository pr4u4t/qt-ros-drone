#ifndef ROSIMAGEPROVIDER_H
#define ROSIMAGEPROVIDER_H

/*------------------------------IMAGE SUBSCRIBER------------------------------*/
class RosImageProvider : public QQuickImageProvider
{
    public:
        RosImageProvider();
        QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
        ImageType imageType() const override { return QQmlImageProviderBase::Image; }
        
    private:
        std::string _topic;
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        image_transport::ImageTransport _it;
        image_transport::Subscriber _sub;
        QImage _last_image;
};

#endif
