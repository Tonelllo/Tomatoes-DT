#include "processcaller.h"
#include <qdebug.h>
#include <qlogging.h>
#include <qprocess.h>
#include <QProcess>
#include <ros/package.h>

ProcessCaller::ProcessCaller()
{
  m_modulePath = QString::fromStdString(ros::package::getPath("tomato_gui"));
}

void ProcessCaller::startHomeMovement()
{
  QProcess* homing = new QProcess(this);
  connect(homing, &QProcess::errorOccurred, this, [homing](){
      qCritical() << "Error occured while starting homing script:";
      qCritical() << homing->errorString();
  });
  connect(homing, &QProcess::readyReadStandardOutput, this, [homing, this](){
      qDebug() << homing->readAllStandardOutput();
      setProcessOutput(homing->readAllStandardOutput());
  });
  homing->start(m_modulePath + "/scripts/tuck_arm.py");
}

QString ProcessCaller::processOutput() const
{
  return m_processOutput;
}

void ProcessCaller::setProcessOutput(const QString& newProcessOutput)
{
  if (m_processOutput == newProcessOutput)
    return;
  m_processOutput = newProcessOutput;
  emit processOutputChanged();
}
