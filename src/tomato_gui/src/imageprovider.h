#ifndef IMAGEPROVIDER_H
#define IMAGEPROVIDER_H

#include <QObject>
#include <QVideoSink>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <QVideoFrame>
#include <qdebug.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


class ImageProvider : public QVideoSink
{
    Q_OBJECT
    Q_PROPERTY(QVideoFrame* maskedImage READ maskedImage WRITE setMaskedImage NOTIFY maskedImageChanged FINAL)
    Q_PROPERTY(QVideoSink* sink READ sink WRITE setsink NOTIFY sinkChanged FINAL)
public:
    explicit ImageProvider(ros::NodeHandle, QObject *parent = nullptr);
    QVideoFrame *maskedImage() const;
    void setMaskedImage(QVideoFrame *newMaskedImage);

    QVideoSink *sink() const;
    void setsink(QVideoSink *newSink);

signals:
    void maskedImageChanged();

    void sinkChanged();

private:
    image_transport::ImageTransport m_image_transport;
    image_transport::Subscriber m_image_sub;
    QVideoFrame *m_maskedImage = nullptr;
    void frameCallback(const sensor_msgs::ImageConstPtr& img);
    QVideoSink *m_sink = nullptr;

public slots:
    void setSink(QVideoSink*);
};

#endif // IMAGEPROVIDER_H
