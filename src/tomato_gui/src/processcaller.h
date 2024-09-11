#ifndef PROCESSCALLER_H
#define PROCESSCALLER_H

#include <QObject>
#include <qobject.h>
#include <qprocess.h>
#include <qtmetamacros.h>

class ProcessCaller : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QString processOutput READ processOutput WRITE setProcessOutput NOTIFY processOutputChanged FINAL)
public:
  ProcessCaller();
  ~ProcessCaller();
  Q_INVOKABLE void startHomeMovement();
  QString processOutput() const;
  void setProcessOutput(const QString& newProcessOutput);
signals:
  void processOutputChanged();

private:
  QProcess* m_homing;
  QString m_processOutput;
  QString m_modulePath;
};

#endif  // PROCESSCALLER_H
