#include "mainwindow.h"
#include <QApplication>
#include <QFile>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    // QFile styleFile(":style/main.qss");
    // if (styleFile.open(QFile::ReadOnly))
    // {
    //     QString styleSheet = QLatin1String(styleFile.readAll());
    //     a.setStyleSheet(styleSheet);
    // }

    w.show();

    return a.exec();
}



// --primary-100:#0085ff;
// --primary-200:#69b4ff;
// --primary-300:#e0ffff;
// --accent-100:#006fff;
// --accent-200:#e1ffff;
// --text-100:#FFFFFF;
// --text-200:#9e9e9e;
// --bg-100:#1E1E1E;
// --bg-200:#2d2d2d;
// --bg-300:#454545;
