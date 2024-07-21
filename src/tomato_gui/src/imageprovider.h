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
    Q_PROPERTY(QVideoSink* origSink READ origSink WRITE setOrigSink NOTIFY origSinkChanged FINAL)
    Q_PROPERTY(QVideoSink* maskedSink READ maskedSink WRITE setMaskedSink NOTIFY maskedSinkChanged FINAL)
public:
    explicit ImageProvider(ros::NodeHandle, QObject *parent = nullptr);
    void setMaskedImage(QVideoFrame *newMaskedImage);

    QVideoSink *origSink() const;

    QVideoSink *maskedSink() const;
    void setMaskedSink(QVideoSink *newMaskedSink);

signals:
    void origSinkChanged();

    void maskedSinkChanged();

private:
    image_transport::ImageTransport m_image_transport;
    image_transport::Subscriber m_image_sub;
    QVideoFrame *m_maskedImage = nullptr;
    void frameCallback(const sensor_msgs::ImageConstPtr& img);
    QVideoSink *m_origSink = nullptr;

    QVideoSink *m_maskedSink = nullptr;

public slots:
    void setOrigSink(QVideoSink*);
};

#endif // IMAGEPROVIDER_H
