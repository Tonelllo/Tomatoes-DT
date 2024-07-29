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
    Q_PROPERTY(uint hue_min READ hue_min WRITE setHue_min NOTIFY hue_minChanged FINAL)
    Q_PROPERTY(uint hue_max READ hue_max WRITE setHue_max NOTIFY hue_maxChanged FINAL)
    Q_PROPERTY(uint sat_min READ sat_min WRITE setSat_min NOTIFY sat_minChanged FINAL)
    Q_PROPERTY(uint sat_max READ sat_max WRITE setSat_max NOTIFY sat_maxChanged FINAL)
    Q_PROPERTY(uint val_min READ val_min WRITE setVal_min NOTIFY val_minChanged FINAL)
    Q_PROPERTY(uint val_max READ val_max WRITE setVal_max NOTIFY val_maxChanged FINAL)
public:
    explicit ImageProvider(ros::NodeHandle, QObject *parent = nullptr);
    void setMaskedImage(QVideoFrame *newMaskedImage);

    QVideoSink *origSink() const;

    QVideoSink *maskedSink() const;
    void setMaskedSink(QVideoSink *newMaskedSink);

    uint hue_min() const;
    void setHue_min(uint newHue_min);

    uint hue_max() const;
    void setHue_max(uint newHue_max);

    uint sat_min() const;
    void setSat_min(uint newSat_min);

    uint sat_max() const;
    void setSat_max(uint newSat_max);

    uint val_min() const;
    void setVal_min(uint newVal_min);

    uint val_max() const;
    void setVal_max(uint newVal_max);

signals:
    void origSinkChanged();

    void maskedSinkChanged();

    void hue_minChanged();

    void hue_maxChanged();

    void sat_minChanged();

    void sat_maxChanged();

    void val_minChanged();

    void val_maxChanged();

private:
    image_transport::ImageTransport m_image_transport;
    image_transport::Subscriber m_image_sub;
    QVideoFrame *m_maskedImage = nullptr;
    void colorSegmentation();
    void frameCallback(const sensor_msgs::ImageConstPtr& img);
    QVideoSink *m_origSink = nullptr;

    QVideoSink *m_maskedSink = nullptr;

    uint m_hue_min = 0;

    uint m_hue_max = 255;

    uint m_sat_min = 0;

    uint m_sat_max = 255;

    uint m_val_min = 0;

    uint m_val_max = 255;

public slots:
    void setOrigSink(QVideoSink*);
};

#endif // IMAGEPROVIDER_H
