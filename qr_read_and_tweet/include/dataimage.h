#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <string>
#include <sstream>


#include <dmtx.h>

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>



#ifndef DATAIMAGE_H
#define DATAIMAGE_H

class DataImage
{
public:
    DataImage();
    DataImage(char *filename);
    void decodificarImagen();

public:
    DmtxImage *imgd;
    IplImage* img2;
    unsigned char  *pxl;
    string imginfo;
};

#endif // DATAIMAGE_H
