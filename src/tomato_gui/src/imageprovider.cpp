#include "imageprovider.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QDateTime>

ImageProvider::ImageProvider(ros::NodeHandle nh, QObject *parent)
    : m_image_transport(nh)
{
    m_image_sub = m_image_transport.subscribe("xtion/rgb/image_raw", 1, &ImageProvider::frameCallback, this);
}

void ImageProvider::frameCallback(const sensor_msgs::ImageConstPtr &img)
{
    // TODO Continuous stream
    // https://stackoverflow.com/questions/10265125/opencv-2-3-convert-mat-to-rgba-pixel-array
    cv::Mat cnv_img, img_hsv, masked_img, mask, origin_converted, masked_conv;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cvPtr->image.copyTo(cnv_img);
    cv::cvtColor(cnv_img, img_hsv, cv::COLOR_BGR2HSV);
    // TODO Declare this variables as atomic
    cv::inRange(img_hsv, cv::Scalar(m_hue_min,m_sat_min,m_val_min), cv::Scalar(m_hue_max, m_sat_max, m_val_max), mask);
    cv::bitwise_and(cnv_img, cnv_img, masked_img, mask);
    cv::cvtColor(masked_img, masked_conv, cv::COLOR_BGR2RGBA);
    cv::cvtColor(cnv_img, origin_converted, cv::COLOR_BGR2RGBA);
    if(m_origSink && m_maskedSink){
        QVideoFrame video_frame(QVideoFrameFormat(QSize(origin_converted.cols, origin_converted.rows),QVideoFrameFormat::Format_RGBA8888));
        QVideoFrame masked_frame(QVideoFrameFormat(QSize(masked_conv.cols, masked_conv.rows),QVideoFrameFormat::Format_RGBA8888));
        if(!video_frame.isValid() || !video_frame.map(QVideoFrame::WriteOnly)){
            qWarning() << "QVideoFrame is not valid or not writable";
            return;
        }
        if(!masked_frame.isValid() || !masked_frame.map(QVideoFrame::WriteOnly)){
            qWarning() << "QVideoFrame is not valid or not writable";
            return;
        }
        QImage::Format image_format = QVideoFrameFormat::imageFormatFromPixelFormat(video_frame.pixelFormat());
        QImage::Format masked_format = QVideoFrameFormat::imageFormatFromPixelFormat(masked_frame.pixelFormat());
        if(image_format == QImage::Format_Invalid){
            qWarning() << "It is not possible to obtain image format from the pixel format of the videoframe";
            return;
        }
        if(masked_format == QImage::Format_Invalid){
            qWarning() << "It is not possible to obtain image format from the pixel format of the videoframe";
            return;
        }
        QImage orig_img((uchar*)origin_converted.data, origin_converted.cols, origin_converted.rows, origin_converted.step, image_format);
        QImage masked_img((uchar*)masked_conv.data, masked_conv.cols, masked_conv.rows, masked_conv.step, masked_format);
        long matSize;
        if(origin_converted.isContinuous()){
            matSize = origin_converted.total() * origin_converted.elemSize();
        }else{
            matSize = origin_converted.step[0] * origin_converted.rows;
        }
        std::memcpy(video_frame.bits(0), orig_img.bits(), orig_img.sizeInBytes());
        std::memcpy(masked_frame.bits(0), masked_img.bits(), masked_img.sizeInBytes());

        video_frame.unmap();
        masked_frame.unmap();
        // Segfault when closing
        m_origSink->setVideoFrame(video_frame);
        m_maskedSink->setVideoFrame(masked_frame);
    }
}

QVideoSink *ImageProvider::origSink() const
{
    return m_origSink;
}

void ImageProvider::setOrigSink(QVideoSink *newSink)
{
    if (m_origSink == newSink)
        return;
    m_origSink = newSink;
    emit origSinkChanged();
}

QVideoSink *ImageProvider::maskedSink() const
{
    return m_maskedSink;
}

void ImageProvider::setMaskedSink(QVideoSink *newMaskedSink)
{
    if (m_maskedSink == newMaskedSink)
        return;
    m_maskedSink = newMaskedSink;
    emit maskedSinkChanged();
}

uint ImageProvider::hue_min() const
{
    return m_hue_min;
}

void ImageProvider::setHue_min(uint newHue_min)
{
    if (m_hue_min == newHue_min)
        return;
    m_hue_min = newHue_min;
    emit hue_minChanged();
}

uint ImageProvider::hue_max() const
{
    return m_hue_max;
}

void ImageProvider::setHue_max(uint newHue_max)
{
    if (m_hue_max == newHue_max)
        return;
    m_hue_max = newHue_max;
    emit hue_maxChanged();
}

uint ImageProvider::sat_min() const
{
    return m_sat_min;
}

void ImageProvider::setSat_min(uint newSat_min)
{
    if (m_sat_min == newSat_min)
        return;
    m_sat_min = newSat_min;
    emit sat_minChanged();
}

uint ImageProvider::sat_max() const
{
    return m_sat_max;
}

void ImageProvider::setSat_max(uint newSat_max)
{
    if (m_sat_max == newSat_max)
        return;
    m_sat_max = newSat_max;
    emit sat_maxChanged();
}

uint ImageProvider::val_min() const
{
    return m_val_min;
}

void ImageProvider::setVal_min(uint newVal_min)
{
    if (m_val_min == newVal_min)
        return;
    m_val_min = newVal_min;
    emit val_minChanged();
}

uint ImageProvider::val_max() const
{
    return m_val_max;
}

void ImageProvider::setVal_max(uint newVal_max)
{
    if (m_val_max == newVal_max)
        return;
    m_val_max = newVal_max;
    emit val_maxChanged();
}

void ImageProvider::colorSegmentation()
{

}
