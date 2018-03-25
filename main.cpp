/**


  @authors Rodrigo Pereira Gonçalves
           Matheus Pires Pimentel

  @version 1.0

*/

#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    mainwindow w;
    w.show();

    return a.exec();
}
