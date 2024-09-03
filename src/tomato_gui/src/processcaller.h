#ifndef PROCESSCALLER_H
#define PROCESSCALLER_H

#include <QObject>
#include <qobject.h>
#include <qtmetamacros.h>

class ProcessCaller : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QString processOutput READ processOutput WRITE setProcessOutput NOTIFY processOutputChanged FINAL)
public:
  ProcessCaller();
  Q_INVOKABLE void startHomeMovement();
  QString processOutput() const;
  void setProcessOutput(const QString& newProcessOutput);
signals:
  void processOutputChanged();

private:
  QString m_processOutput;
  QString m_modulePath;
};

#endif  // PROCESSCALLER_H
