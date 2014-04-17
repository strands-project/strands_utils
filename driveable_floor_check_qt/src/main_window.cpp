//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 5.12.2012
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <QApplication>
#include <QPainter>
#include <iostream>
#include "../include/DriveableFloorCheckQt/main_window.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinect {
using namespace Qt;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MY_IMG_X 640
#define MY_IMG_Y 512
#define MY_BUT_X 128
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Draws the QImage
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::paintEvent(QPaintEvent *event)
{
  QRect q((8 + MY_BUT_X + 8), 8, MY_IMG_X, MY_IMG_Y);
  QPainter p;

  p.begin(this);
  p.drawImage(q, *pImage);
  p.end();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QWidget(parent), qnode(argc, argv)
{

// Initially there are no resources allocated.
  ImageBuffer = 0;
  pImage = 0;
  pConnect = 0;
  pShow = 0;
  bConnected = false;

// Set up the main widget.
  setFixedSize((8 + MY_BUT_X + 8 + MY_IMG_X + 8), (8 + MY_IMG_Y + 8));
  setWindowTitle("Qt-based ROS main window");

// Create a buffer for the QImage.
  ImageBuffer = new uint8_t[MY_IMG_X * MY_IMG_Y * 4];

// Hand the buffer pointer also to the subscriber thread.
  qnode.pBuffer = ImageBuffer;
  qnode.DispStep = (MY_IMG_X * 4);

// Create a new QImage.
  pImage = new QImage(ImageBuffer, MY_IMG_X, MY_IMG_Y, (MY_IMG_X * 4), QImage::Format_RGB32);

// Create the button to start the ROS thing.
  pConnect = new QPushButton(tr("Connect"), this);
  pConnect->setFont(QFont("System", 12, QFont::Bold));
  pConnect->setGeometry(8, 8, MY_BUT_X, 32);

// Create the checkbox.
  pShow = new QCheckBox(tr("Show"), this);
  pShow->setFont(QFont("System", 12, QFont::Bold));
  pShow->setGeometry(8, 48, MY_BUT_X, 32);

// Connect the signals and slots.
  QObject::connect(pConnect, SIGNAL(clicked()), this, SLOT(connectRequest()));
  QObject::connect(pShow, SIGNAL(toggled(bool)), this, SLOT(showRequest(bool)));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(imageUpdated()), this, SLOT(updateQImage()));
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MainWindow::~MainWindow()
{

// Delete the checkbox.
  if (pShow != 0)
  {
    delete pShow;
    pShow = 0;
  }

// Delete the connect button.
  if (pConnect != 0)
  {
    delete pConnect;
    pConnect = 0;
  }

// Delete the QImage.
  if (pImage != 0)
  {
    delete pImage;
    pImage = 0;
  }

// Delete the image buffer.
  if (ImageBuffer != 0)
  {
    delete [] ImageBuffer;
    ImageBuffer = 0;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::connectRequest()
{

// Start the ROS worker thread.
  if (!bConnected)
  {
    qnode.init();
    bConnected = true;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::showRequest(bool checked)
{
  qnode.bShowResults = checked;
  printf("bShowResults = %i\n", qnode.bShowResults);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::updateQImage()
{
  update();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

