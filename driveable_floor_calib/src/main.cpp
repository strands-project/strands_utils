//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 13.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <QtGui>
#include <QApplication>
#include "../include/DriveableFloorCalib/main_window.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  TopKinectFloorCalib::MainWindow w(argc, argv);

  w.show();
  int result = app.exec();

  return result;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

