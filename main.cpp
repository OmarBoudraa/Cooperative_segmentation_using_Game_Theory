#include "principale.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    principale w;
    w.show();

    return a.exec();
}
