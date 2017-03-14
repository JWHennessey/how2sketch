#ifndef MAIN_CPP
#define MAIN_CPP


#include <QApplication>
#include "widgets/mainwindow.h"

int main(int argc, char *argv[])
{
    auto format = QSurfaceFormat();
    format.setVersion(4,3);
    format.setSamples(4);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.resize(1200, 800);
    return a.exec();
}

#endif
