//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 5.12.2012
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef TopKinect_MAIN_WINDOW_H
#define TopKinect_MAIN_WINDOW_H
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <stdint.h>
#include <QObject>
#include <QWidget>
#include <QPushButton>
#include <QImage>
#include <QCheckBox>
#include "qnode.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinect {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class MainWindow : public QWidget
{
  Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

  public Q_SLOTS:
    void updateQImage();

  private Q_SLOTS:
    void connectRequest(void);
    void showRequest(bool checked);

  private:
    QNode qnode;

    bool bConnected;
    uint8_t *ImageBuffer;
    QImage *pImage;
    QPushButton *pConnect;
    QCheckBox *pShow;

    void paintEvent(QPaintEvent *event);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

