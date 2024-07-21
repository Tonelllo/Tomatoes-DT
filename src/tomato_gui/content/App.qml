// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only

import QtQuick 6.2
import tomato_gui

Window {
    width: mainScreen.width
    height: mainScreen.height

    visible: true
    title: "tomato_gui"

    Screen01 {
        id: mainScreen
    }
}

