#ifndef TOMATOESLISTMODEL_H
#define TOMATOESLISTMODEL_H

#include <qabstractitemmodel.h>
#include <qnamespace.h>
#include <qtmetamacros.h>
#include <QAbstractListModel>
#include <QObject>
#include <QQmlEngine>
#include <QVector3D>
#include <ros/ros.h>
#include "ros/node_handle.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/subscriber.h"

class TomatoesListModel : public QAbstractListModel
{
  Q_OBJECT
  QML_ELEMENT
  Q_PROPERTY(int ripeness READ ripeness WRITE setRipeness NOTIFY ripenessChanged FINAL)
  enum Roles
  {
    xPos = Qt::UserRole + 1,
    yPos,
    zPos,
  };

public:
  static std::shared_ptr<ros::NodeHandle> nodeHandle;
  TomatoesListModel();
  int ripeness() const;
  void setRipeness(int newRipeness);
  virtual QHash<int, QByteArray> roleNames() const override;
  QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  Q_INVOKABLE void rosStart();
signals:
  void ripenessChanged();

private:
  ros::Subscriber tomatoPositionSubscriber;
  void tomatoInsertionCallback(const geometry_msgs::PoseArrayConstPtr&);
  int m_ripeness;
  QVector<QVector3D> m_points;
};

#endif  // TOMATOESLISTMODEL_H
