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
  connect(homing, &QProcess::errorOccurred, this, [homing]() {
    qCritical() << "Error occured while starting homing script:";
    qCritical() << homing->errorString();
  });
  connect(homing, &QProcess::readyReadStandardOutput, this, [homing, this]() {
    QString out = homing->readAllStandardOutput();
    qDebug() << out;
    setProcessOutput(out);
  });
  QStringList arguments;
  arguments << m_modulePath + "/scripts/tuck_arm.py";
  homing->start("python", arguments);
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
