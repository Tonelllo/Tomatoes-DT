#include "processcaller.h"
#include <qdebug.h>
#include <qlogging.h>
#include <qprocess.h>
#include <QProcess>
#include <ros/package.h>
#include <QCoreApplication>

ProcessCaller::ProcessCaller()
{
  m_modulePath = QString::fromStdString(ros::package::getPath("tomato_gui"));
  if (m_modulePath.isEmpty())
  {
    qDebug() << "tomato_gui not present defaulting to local directory";
    m_modulePath = QCoreApplication::applicationDirPath()+"/..";
  }
  m_homing = new QProcess(this);
}

ProcessCaller::~ProcessCaller()
{
  delete m_homing;
}

void ProcessCaller::startHomeMovement()
{
  QProcess* m_homing = new QProcess(this);
  connect(m_homing, &QProcess::errorOccurred, this, [m_homing]() {
    qCritical() << "Error occured while starting m_homing script:";
    qCritical() << m_homing->errorString();
  });
  connect(m_homing, &QProcess::readyReadStandardOutput, this, [m_homing, this]() {
    QString out = m_homing->readAllStandardOutput();
    qDebug() << out;
    setProcessOutput(out);
  });
  QStringList arguments;
  arguments << m_modulePath + "/scripts/tuck_arm.py";
  m_homing->start("python", arguments);
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
