// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only

#include <QGuiApplication>
#include <QQmlApplicationEngine>

#include "app_environment.h"
#include "imageprovider.h"
#include "import_qml_components_plugins.h"
#include "import_qml_plugins.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ImageProvider");
    ros::AsyncSpinner spinner(4);
    ros::NodeHandle nh;
    spinner.start();

    set_qt_environment();
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    ImageProvider *imageProvider = new ImageProvider(nh, &app);
    qmlRegisterSingletonInstance("Tonelllo.ImageProvider", 1, 0, "ImageProvider", imageProvider);
    const QUrl url(u"qrc:/qt/qml/Main/main.qml"_qs);
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreated,
        &app,
        [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        },
        Qt::QueuedConnection);

    engine.addImportPath(QCoreApplication::applicationDirPath() + "/qml");
    engine.addImportPath(":/");

    engine.load(url);

    if (engine.rootObjects().isEmpty()) {
        return -1;
    }

    return app.exec();
}
