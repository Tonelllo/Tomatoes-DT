// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only
// catkin_make -C ../../ && cp ../../build/compile_commands.json ../../.qtc_clangd/compile_commands.json

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <memory>

#include "ros/node_handle.h"
#include "tomatoeslistmodel.h"
#include "app_environment.h"
#include "imageprovider.h"
#include "import_qml_components_plugins.h"
#include "import_qml_plugins.h"
#include "processcaller.h"
#include <qqml.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ImageProvider");
    ros::AsyncSpinner spinner(10);
    ros::NodeHandle nh;
    spinner.start();

    set_qt_environment();
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;
    ImageProvider *imageProvider = new ImageProvider(nh, &app);
    ProcessCaller *processCaller = new ProcessCaller();
    qmlRegisterSingletonInstance("Tonelllo.ImageProvider", 1, 0, "ImageProvider", imageProvider);
    qmlRegisterSingletonInstance("Tonelllo.ProcessCaller", 1, 0, "ProcessCaller", processCaller);
    TomatoesListModel::nodeHandle = std::make_shared<ros::NodeHandle>(nh);
    qmlRegisterType<TomatoesListModel>("Tonelllo.TomatoModels", 1, 0, "TomatoListModel");
    // TomatoesListModel *t = new TomatoesListModel();
    // qmlRegisterSingletonInstance("Tonelllo.TomatoListModel", 1, 0, "TomatoListModel", t);
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
