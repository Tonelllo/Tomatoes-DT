#include "imageprovider.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QDateTime>

ImageProvider::ImageProvider(ros::NodeHandle nh, QObject *parent)
    : m_image_transport(nh)
{
    m_image_sub = m_image_transport.subscribe("xtion/rgb/image_raw", 1, &ImageProvider::frameCallback, this);
}

QVideoFrame *ImageProvider::maskedImage() const
{
    return m_maskedImage;
}

void ImageProvider::setMaskedImage(QVideoFrame *newMaskedImage)
{
    if (m_maskedImage == newMaskedImage)
        return;
    m_maskedImage = newMaskedImage;
    emit maskedImageChanged();
}

void ImageProvider::frameCallback(const sensor_msgs::ImageConstPtr &img)
{
    // TODO Continuous stream
    // https://stackoverflow.com/questions/10265125/opencv-2-3-convert-mat-to-rgba-pixel-array
    cv::Mat cnv_img, img_hsv, masked_img, mask, converted;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cvPtr->image.copyTo(cnv_img);
    cv::cvtColor(cnv_img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(0,0,0), cv::Scalar(100, 100, 100), mask);
    cv::bitwise_and(cnv_img, cnv_img, masked_img, mask);
    // masked_img.convertTo(converted, QImage::Format_ARGB32);
    cv::cvtColor(cnv_img, converted, cv::COLOR_BGR2RGBA);
    if(m_sink){
        QVideoFrame video_frame(QVideoFrameFormat(QSize(converted.cols, converted.rows),QVideoFrameFormat::Format_RGBA8888));
        if(!video_frame.isValid() || !video_frame.map(QVideoFrame::WriteOnly)){
            qWarning() << "QVideoFrame is not valid or not writable";
            return;
        }
        QImage::Format image_format = QVideoFrameFormat::imageFormatFromPixelFormat(video_frame.pixelFormat());
        qDebug() << image_format;
        if(image_format == QImage::Format_Invalid){
            qWarning() << "It is not possible to obtain image format from the pixel format of the videoframe";
            return;
        }
        int plane = 0;
        // QImage image(video_frame.bits(plane), video_frame.width(),video_frame.height(), image_format);
        // image.fill(QColor::fromRgb(QRandomGenerator::global()->generate()));
        QImage cane((uchar*)converted.data, converted.cols, converted.rows, converted.step, image_format);
        // QImage::Format_RGB32
        long matSize;
        if(converted.isContinuous()){
            qDebug() << "continuous";
            matSize = converted.total() * converted.elemSize();
        }else{
            qDebug() << "not continuous";
            matSize = converted.step[0] * converted.rows;
        }
        if(video_frame.bits(0) != nullptr)
            std::memcpy(video_frame.bits(0), cane.bits(), cane.sizeInBytes());

        video_frame.unmap();
        // Segfault when closing
        m_sink->setVideoFrame(video_frame);
    }
}

void ImageProvider::setSink(QVideoSink *sink)
{
    m_sink = sink;
}

QVideoSink *ImageProvider::sink() const
{
    return m_sink;
}

void ImageProvider::setsink(QVideoSink *newSink)
{
    if (m_sink == newSink)
        return;
    m_sink = newSink;
    emit sinkChanged();
}
