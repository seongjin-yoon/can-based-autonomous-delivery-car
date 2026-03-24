#include "mainwindow.h"
#include <QApplication>
#include <QStyleFactory>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    // Qt6 스타일 설정
    app.setStyle(QStyleFactory::create("Fusion"));
    
    MainWindow w;
    w.show();
    return app.exec();
}
