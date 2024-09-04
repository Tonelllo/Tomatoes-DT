#include "imageprovider.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QDateTime>
#include <cstdint>
#include <cstdlib>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <toml++/toml.hpp>
#include <fstream>
#include <qdebug.h>
#include <qlogging.h>
#include <ros/package.h>
#include <QDir>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

ImageProvider::ImageProvider(ros::NodeHandle nh, QObject* parent) : m_image_transport(nh), m_yolo_transport(nh)
{
  m_image_sub = m_image_transport.subscribe("xtion/rgb/image_raw", 1, &ImageProvider::frameCallback, this);
  m_yolo_sub = m_image_transport.subscribe("tomato_detection/detection_result", 1, &ImageProvider::yoloCallback, this);
  QString modulePath = QString::fromStdString(ros::package::getPath("tomato_gui"));
  if (!QDir(modulePath + "/VisionConfig").exists())
  {
    QDir().mkdir(modulePath + "/VisionConfig");
  }

  setSavePath(modulePath + "/VisionConfig/config.toml");
}

void ImageProvider::setFrame(QVideoSink* sink, cv::Mat image)
{
  if (sink)
  {
    QVideoFrame video_frame(QVideoFrameFormat(QSize(image.cols, image.rows), QVideoFrameFormat::Format_RGBA8888));
    if (!video_frame.isValid() || !video_frame.map(QVideoFrame::WriteOnly))
    {
      qWarning() << "QVideoFrame is not valid or not writable";
      return;
    }
    QImage::Format image_format = QVideoFrameFormat::imageFormatFromPixelFormat(video_frame.pixelFormat());
    if (image_format == QImage::Format_Invalid)
    {
      qWarning() << "It is not possible to obtain image format from the pixel format of the videoframe";
      return;
    }
    QImage orig_img((uchar*)image.data, image.cols, image.rows, image.step, image_format);
    long matSize;
    if (image.isContinuous())
    {
      matSize = image.total() * image.elemSize();
    }
    else
    {
      matSize = image.step[0] * image.rows;
    }
    std::memcpy(video_frame.bits(0), orig_img.bits(), orig_img.sizeInBytes());
    video_frame.unmap();
    sink->setVideoFrame(video_frame);
  }
}

void ImageProvider::yoloCallback(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cvPtr;
  cv::Mat cnvImg, qmlImg;
  cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  cvPtr->image.copyTo(cnvImg);
  cv::cvtColor(cnvImg, qmlImg, cv::COLOR_BGR2RGBA);

  setFrame(m_yoloSink, qmlImg);
}

void ImageProvider::frameCallback(const sensor_msgs::ImageConstPtr& img)
{
  // TODO Continuous stream
  // https://stackoverflow.com/questions/10265125/opencv-2-3-convert-mat-to-rgba-pixel-array
  cv::Mat cnv_img, img_hsv, masked_img, mask, origin_converted, masked_conv, circled_img, circled_conv;
  cv_bridge::CvImagePtr cvPtr;
  cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  cvPtr->image.copyTo(cnv_img);
  cv::cvtColor(cnv_img, img_hsv, cv::COLOR_BGR2HSV);
  // TODO Declare this variables as atomic
  cv::inRange(img_hsv, cv::Scalar(m_hue_min, m_sat_min, m_val_min), cv::Scalar(m_hue_max, m_sat_max, m_val_max), mask);
  cv::bitwise_and(cnv_img, cnv_img, masked_img, mask);

  cv::cvtColor(cnv_img, origin_converted, cv::COLOR_BGR2RGBA);
  cv::cvtColor(masked_img, masked_conv, cv::COLOR_BGR2RGBA);
  circled_img = colorSegmentation(masked_conv, cnv_img);
  cv::cvtColor(circled_img, circled_conv, cv::COLOR_BGR2RGBA);

  setFrame(m_origSink, origin_converted);
  setFrame(m_maskedSink, masked_conv);
  setFrame(m_radSink, circled_conv);
}

void ImageProvider::saveSettings()
{
  auto toSave = toml::table{ { "hue",
                               toml::table{
                                   { "min", m_hue_min },
                                   { "max", m_hue_max },
                               } },
                             { "sat",
                               toml::table{
                                   { "min", m_sat_min },
                                   { "max", m_sat_max },
                               } },
                             { "val",
                               toml::table{
                                   { "min", m_val_min },
                                   { "max", m_val_max },
                               } },
                             { "rad", toml::table{
                                          { "min", m_rad_min },
                                          { "max", m_rad_max },
                                      } } };
  std::ofstream outFile((m_savePath).toStdString());
  outFile << toSave;
}

void ImageProvider::restoreSettings()
{
  toml::table toRestore = toml::parse_file(m_savePath.toStdString());

  uint hueMin = toRestore["hue"]["min"].value_or(ERROR_VAL);
  if (hueMin == ERROR_VAL)
  {
    qCritical() << "Hue not read correctly";
    exit(EXIT_FAILURE);
  }
  setHue_min(hueMin);

  uint hueMax = toRestore["hue"]["max"].value_or(ERROR_VAL);
  if (hueMax == ERROR_VAL)
  {
    qCritical() << "Hue not read correctly";
    exit(EXIT_FAILURE);
  }
  setHue_max(hueMax);

  uint satMin = toRestore["sat"]["min"].value_or(ERROR_VAL);
  if (satMin == ERROR_VAL)
  {
    qCritical() << "Sat not read correctly";
    exit(EXIT_FAILURE);
  }
  setSat_min(satMin);

  uint satMax = toRestore["sat"]["max"].value_or(ERROR_VAL);
  if (satMax == ERROR_VAL)
  {
    qCritical() << "Sat not read correctly";
    exit(EXIT_FAILURE);
  }
  setSat_max(satMax);

  uint valMin = toRestore["val"]["min"].value_or(ERROR_VAL);
  if (valMin == ERROR_VAL)
  {
    qCritical() << "Val not read correctly";
    exit(EXIT_FAILURE);
  }
  setVal_min(valMin);

  uint valMax = toRestore["val"]["max"].value_or(ERROR_VAL);
  if (valMax == ERROR_VAL)
  {
    qCritical() << "Val not read correctly";
    exit(EXIT_FAILURE);
  }
  setVal_max(valMax);

  uint radMin = toRestore["rad"]["min"].value_or(ERROR_VAL);
  if (radMin == ERROR_VAL)
  {
    qCritical() << "Rad not read correctly";
    exit(EXIT_FAILURE);
  }
  setRad_min(radMin);

  uint radMax = toRestore["rad"]["max"].value_or(ERROR_VAL);
  if (radMax == ERROR_VAL)
  {
    qCritical() << "Rad not read correctly";
    exit(EXIT_FAILURE);
  }
  setRad_max(radMax);
}

QVideoSink* ImageProvider::origSink() const
{
  return m_origSink;
}

void ImageProvider::setOrigSink(QVideoSink* newSink)
{
  if (m_origSink == newSink)
    return;
  m_origSink = newSink;
  emit origSinkChanged();
}

QVideoSink* ImageProvider::maskedSink() const
{
  return m_maskedSink;
}

void ImageProvider::setMaskedSink(QVideoSink* newMaskedSink)
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

cv::Mat ImageProvider::colorSegmentation(cv::Mat img, cv::Mat orig)
{
  cv::Mat blurred;
  cv::cvtColor(img, blurred, cv::COLOR_BGR2GRAY);
  cv::medianBlur(blurred, blurred, 15);
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1, static_cast<float>(blurred.rows) / 8, 30, 10, m_rad_min,
                   m_rad_max);
  for (cv::Vec3i c : circles)
  {
    cv::Point center = cv::Point(c[0], c[1]);
    // circle center
    cv::circle(orig, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
    // circle outline
    int radius = c[2];
    cv::circle(orig, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
  }
  return orig;
}

uint ImageProvider::rad_min() const
{
  return m_rad_min;
}

void ImageProvider::setRad_min(uint newRad_min)
{
  if (m_rad_min == newRad_min)
    return;
  m_rad_min = newRad_min;
  emit rad_minChanged();
}

uint ImageProvider::rad_max() const
{
  return m_rad_max;
}

void ImageProvider::setRad_max(uint newRad_max)
{
  if (m_rad_max == newRad_max)
    return;
  m_rad_max = newRad_max;
  emit rad_maxChanged();
}

QVideoSink* ImageProvider::radSink() const
{
  return m_radSink;
}

void ImageProvider::setRadSink(QVideoSink* newRadSink)
{
  if (m_radSink == newRadSink)
    return;
  m_radSink = newRadSink;
  emit radSinkChanged();
}

QString ImageProvider::savePath() const
{
  return m_savePath;
}

void ImageProvider::setSavePath(const QString& newSavePath)
{
  if (m_savePath == newSavePath)
    return;
  m_savePath = newSavePath;
  emit savePathChanged();
}

QVideoSink *ImageProvider::yoloSink() const
{
  return m_yoloSink;
}

void ImageProvider::setYoloSink(QVideoSink *newYoloSink)
{
  if (m_yoloSink == newYoloSink)
    return;
  m_yoloSink = newYoloSink;
  emit yoloSinkChanged();
}
