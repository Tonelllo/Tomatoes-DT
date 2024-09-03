cmake_minimum_required(VERSION 3.16.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(catkin_DIR "/opt/ros/noetic/share/catkin/cmake/")
set(Qt6_DIR "/home/tonello/Qt/6.7.2/gcc_64/lib/cmake/Qt6/")

option(LINK_INSIGHT "Link Qt Insight Tracker library" ON)
option(BUILD_QDS_COMPONENTS "Build design studio components" ON)

project(tomato_guiApp LANGUAGES CXX)
set(CMAKE_PROJECT_NAME tomato_guiApp )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include(FetchContent)
FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(tomlplusplus)

find_package(Qt6 6.2 REQUIRED COMPONENTS Core Gui Qml Quick Multimedia)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    cv_bridge
)
find_package(OpenCV REQUIRED COMPONENTS
    opencv_core
    opencv_highgui
)

if (Qt6_VERSION VERSION_GREATER_EQUAL 6.3)
    qt_standard_project_setup()
endif()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

qt_add_executable(tomato_guiApp
    src/main.cpp
    src/imageprovider.h
    src/imageprovider.cpp
    src/processcaller.h
    src/processcaller.cpp
)

qt_add_resources(tomato_guiApp "configuration"
    PREFIX "/"
    FILES
        qtquickcontrols2.conf
)

target_link_libraries(tomato_guiApp PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
    Qt6::Multimedia
    PUBLIC
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    tomlplusplus::tomlplusplus
)

set(QT_QML_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/qml)
set(QML_IMPORT_PATH ${QT_QML_OUTPUT_DIRECTORY}
    CACHE STRING "Import paths for Qt Creator's code model"
    FORCE
)

if (BUILD_QDS_COMPONENTS)
    include(${CMAKE_CURRENT_SOURCE_DIR}/qmlcomponents)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/qmlmodules)

if (LINK_INSIGHT)
    include(${CMAKE_CURRENT_SOURCE_DIR}/insight)
endif ()

include(GNUInstallDirs)
install(TARGETS tomato_guiApp
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# make IDEs aware of the QML import path
set(QML_IMPORT_PATH ${PROJECT_BINARY_DIR}/qml CACHE PATH
  ttofuf  "Path to the custom QML components defined by the project")
