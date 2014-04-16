//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 5.12.2012
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <QtGui>
#include <QApplication>
#include "../include/DriveableFloorCheckConsole/qnode.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char **argv)
{
  //QApplication app(argc, argv);
  //TopKinect::MainWindow w(argc,argv);

  TopKinect::QNode qnode(argc, argv);
  qnode.init();

  //w.show();
  //int result = app.exec();

  {

        qnode.run();

  }

  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

