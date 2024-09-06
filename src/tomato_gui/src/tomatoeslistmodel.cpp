#include "tomatoeslistmodel.h"
#include <qabstractitemmodel.h>
#include <qdebug.h>
#include <qvectornd.h>
#include <memory>
#include "geometry_msgs/Pose.h"
#include "ros/subscriber.h"

std::shared_ptr<ros::NodeHandle> TomatoesListModel::nodeHandle = nullptr;

TomatoesListModel::TomatoesListModel()
{
  rosStart();
}

int TomatoesListModel::ripeness() const
{
  return m_ripeness;
}

void TomatoesListModel::setRipeness(int newRipeness)
{
  if (m_ripeness == newRipeness)
    return;
  m_ripeness = newRipeness;
  emit ripenessChanged();
}

QHash<int, QByteArray> TomatoesListModel::roleNames() const
{
  QHash<int, QByteArray> ret;
  ret[xPos] = "xPos";
  ret[yPos] = "yPos";
  ret[zPos] = "zPos";
  return ret;
}

int TomatoesListModel::rowCount(const QModelIndex& parent) const
{
  return m_points.length();
}

QVariant TomatoesListModel::data(const QModelIndex& index, int role) const
{
  if (!index.isValid() || index.row() < 0 || index.row() >= m_points.length())
    return QVariant();

  QVector3D point = m_points[index.row()];
  switch ((Roles)role)
  {
    case xPos:
        return point.x();
      break;
    case yPos:
        return point.y();
      break;
    case zPos:
        return point.z();
      break;
  }
  return {};
}

void TomatoesListModel::tomatoInsertionCallback(const geometry_msgs::PoseArrayConstPtr& pa)
{
  beginRemoveRows(QModelIndex(), 0, m_points.size());
  m_points.clear();
  endRemoveRows();
  beginInsertRows(QModelIndex(), 0, pa->poses.size());
  for (const geometry_msgs::Pose& pos : pa->poses)
  {
    if (static_cast<int>(pos.orientation.w) == m_ripeness)
    {
      QVector3D newElem = { static_cast<float>(pos.orientation.x), static_cast<float>(pos.orientation.y),
                            static_cast<float>(pos.orientation.z) };
      m_points.push_back(newElem);
    }
  }
  endInsertRows();
}

void TomatoesListModel::rosStart()
{
  qDebug() << "called rosStart";
  // tomatoPositionSubscriber = nodeHandle->subscribe("/tomato_detection/detected_tomatoes", 1,
  //                                                  &TomatoesListModel::tomatoInsertionCallback, this);
  tomatoPositionSubscriber = nodeHandle->subscribe("/tomato_vision_manager/tomato_position", 1,
                                                   &TomatoesListModel::tomatoInsertionCallback, this);
  if (!tomatoPositionSubscriber)
  {
    ROS_ERROR("UNABLE TO CREATE SUBSCRIBER");
  }
}
