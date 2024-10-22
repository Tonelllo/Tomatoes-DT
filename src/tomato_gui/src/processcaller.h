#ifndef PROCESSCALLER_H
#define PROCESSCALLER_H

#include <QObject>
#include "ros/node_handle.h"
#include <qobject.h>
#include <qprocess.h>
#include <qtmetamacros.h>
#include <ros/ros.h>

class ProcessCaller : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QString processOutput READ processOutput WRITE setProcessOutput NOTIFY processOutputChanged FINAL)
public:
  ProcessCaller();
  ~ProcessCaller();
  Q_INVOKABLE void startHomeMovement();
  Q_INVOKABLE void stopMovement();
  QString processOutput() const;
  void setProcessOutput(const QString& newProcessOutput);
signals:
  void processOutputChanged();

private:
  QProcess* m_homing;
  QString m_processOutput;
  QString m_modulePath;
  ros::NodeHandle m_nh;
  ros::ServiceClient stopClient;
};

#endif  // PROCESSCALLER_H
