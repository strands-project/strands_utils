//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 20.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <QApplication>
#include <QPainter>
#include <iostream>
#include "../include/DriveableFloorCalib/main_window.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinectFloorCalib {
using namespace Qt;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Size of the Qt window and width of the push buttons.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MY_IMG_X 640
#define MY_IMG_Y 512
#define MY_BUT_X 128
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Draws the QImage.
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
// Constructor of the class 'MainWindow'. Initialises member attri-
// butes and allocates the required resources and the elements of the
// Qt window.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QWidget(parent), qnode(argc, argv)
{

// Initially there are no resources allocated.
  ImageBuffer = 0;
  pImage = 0;
  pConnect = 0;
  pNominal = 0;
  pMaximum = 0;
  pMinimum = 0;
  pSave = 0;
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

// Create the button that saves the v-disparity nominal values.
  pNominal = new QPushButton(tr("Nominal"), this);
  pNominal->setFont(QFont("System", 12, QFont::Bold));
  pNominal->setGeometry(8, 48, MY_BUT_X, 32);

// Create the button that saves the v-disparity maximum values.
  pMaximum = new QPushButton(tr("Maximum"), this);
  pMaximum->setFont(QFont("System", 12, QFont::Bold));
  pMaximum->setGeometry(8, 88, MY_BUT_X, 32);

// Create the button that saves the v-disparity minimum values.
  pMinimum = new QPushButton(tr("Minimum"), this);
  pMinimum->setFont(QFont("System", 12, QFont::Bold));
  pMinimum->setGeometry(8, 128, MY_BUT_X, 32);

// Create the button that saves the v-disparity values to the file "Params.txt".
  pSave = new QPushButton(tr("Save"), this);
  pSave->setFont(QFont("System", 12, QFont::Bold));
  pSave->setGeometry(8, 168, MY_BUT_X, 32);

// Connect the signals and slots.
  QObject::connect(pConnect, SIGNAL(clicked()), this, SLOT(connectRequest()));
  QObject::connect(pNominal, SIGNAL(clicked()), this, SLOT(nominalRequest()));
  QObject::connect(pMinimum, SIGNAL(clicked()), this, SLOT(minimumRequest()));
  QObject::connect(pMaximum, SIGNAL(clicked()), this, SLOT(maximumRequest()));
  QObject::connect(pSave, SIGNAL(clicked()), this, SLOT(saveRequest()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(imageUpdated()), this, SLOT(updateQImage()));
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor of the class 'MainWindow'. Releases the resources that
// had been allocated by the constructor.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
MainWindow::~MainWindow()
{

// Delete the save button
  if (pSave != 0)
  {
    delete pSave;
    pSave = 0;
  }

// Delete the minimum button
  if (pMinimum != 0)
  {
    delete pMinimum;
    pMinimum = 0;
  }

// Delete the maximum button
  if (pMaximum != 0)
  {
    delete pMaximum;
    pMaximum = 0;
  }

// Delete the nominal button
  if (pNominal != 0)
  {
    delete pNominal;
    pNominal = 0;
  }

// Delete the connect button
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
// Method invoked when the push button 'Connect' is pressed. Starts
// the Qt thread that subscribes to the depth camera's disparity ROS
// topic.
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
// Method invoked when the push button 'Nominal' is pressed. Stores
// the current line parameters (k, d) computed from the v-disparity
// image as nominal line parameters. Also, the plane parameters com-
// puted from the 3D points corresponding to disparity values of the
// ground are stored.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::nominalRequest()
{
  printf("Saving current values as nominal values (%f | %f)\n", qnode.k_disp, qnode.d_disp);
  k_nom = qnode.k_disp;
  d_nom = qnode.d_disp;
  nx = qnode.nx_plane;
  ny = qnode.ny_plane;
  nz = qnode.nz_plane;
  d = qnode.d_plane;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Method invoked when the push button 'Maximum' is pressed. Stores
// the current line parameters (k, d) computed from the v-disparity
// image as upper boundary for the line parameters.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::maximumRequest()
{
  printf("Saving current values as maximum values (%f | %f)\n", qnode.k_disp, qnode.d_disp);
  k_max = qnode.k_disp;
  d_max = qnode.d_disp;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Method invoked when the push button 'Minimum' is pressed. Stores
// the current line parameters (k, d) computed from the v-disparity
// image as lower boundary for the line parameters.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::minimumRequest()
{
  printf("Saving current values as minimum values (%f | %f)\n", qnode.k_disp, qnode.d_disp);
  k_min = qnode.k_disp;
  d_min = qnode.d_disp;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Method invoked when the push button 'Save' is pressed. Writes the
// stored line and plane parameters to a file 'Params.txt'
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::saveRequest()
{
  printf("Writing values to file\n");
  FILE *fp = fopen("Params.txt", "w");
  if (fp != 0)
  {
    fprintf(fp, "ImgW = 640.0\n");
    fprintf(fp, "ImgH = 480.0\n");
    fprintf(fp, "MinK = %f\n", k_min);
    fprintf(fp, "MinD = %f\n", d_min);
    fprintf(fp, "NomK = %f\n", k_nom);
    fprintf(fp, "NomD = %f\n", d_nom);
    fprintf(fp, "MaxK = %f\n", k_max);
    fprintf(fp, "MaxD = %f\n", d_max);
    fprintf(fp, "PlNx = %f\n", nx);
    fprintf(fp, "PlNy = %f\n", ny);
    fprintf(fp, "PlNz = %f\n", nz);
    fprintf(fp, "PlDi = %f\n", d);

    fclose(fp);
  }
  else printf(" - failed to create file 'Params.txt'\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Method invoked by draw events.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::updateQImage()
{
  update();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

