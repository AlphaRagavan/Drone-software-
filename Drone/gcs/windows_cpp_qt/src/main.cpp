#include "main_window.hpp"
#include <QApplication>
#include <QFile>

int main(int argc, char* argv[]) {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#endif

    QApplication app(argc, argv);
    app.setApplicationName("NIDAR AirMouse Ground Control Station");
    app.setOrganizationName("NIDAR Robotics");

    MainWindow window;
    window.show();

    return app.exec();
}
