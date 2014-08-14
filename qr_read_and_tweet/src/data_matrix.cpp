/*******************************************************
*                                                      *
* Compiled using  gcc, tested on Ubuntu 10.4 LTS.      *
*                                                      *
* By Jaime Pulido Fentanes, 2012.                      *
*******************************************************/



#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>


#include <ros/ros.h>
//#include <dmtx.h>

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include "dataimage.h"


DataImage *decimg;
ros::Publisher dtmtxmsg_pub;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  printf("-");
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);
    cv::imwrite("/tmp/bridgeimage.jpg", cv_ptr->image, compression_params);
    char filen[100];
    sprintf(filen,"/tmp/bridgeimage.jpg");
    decimg = new DataImage(filen);
    decimg->decodificarImagen();
    std_msgs::String dtmtxmsg;
    dtmtxmsg.data = decimg->imginfo.data;
    dtmtxmsg_pub.publish(dtmtxmsg);
    delete decimg;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}




int main(int argc, char *argv[])
{
  char filename[50];
  int i=1;
  bool from_file=false;
  //bool from_cam=false;

  strcpy(filename, "./Images/test3.jpeg");

  while(i<argc)
  {
    if(!strcmp(argv[i],"-f")){  strcpy(filename, argv[i+1]);    from_file=true;    }
    //if(!strcmp(argv[i],"-d")){ from_cam=true;   }
    i++;
  }

  ros::init(argc, argv, "DataMatrix_Node");
  ros::NodeHandle dn;



  image_transport::ImageTransport it(dn);
  image_transport::Subscriber imsub = it.subscribe("/head_xtion/rgb/image_color", 1, imageCallback);

  dtmtxmsg_pub = dn.advertise<std_msgs::String>("/datamatrix/msg", 1);



  if(from_file)
  {
      decimg = new DataImage(filename);
      decimg->decodificarImagen();
      std_msgs::String dtmtxmsg;
      dtmtxmsg.data = decimg->imginfo.data;
      dtmtxmsg_pub.publish(dtmtxmsg);
      delete decimg;
  }

 // if(from_cam)  capturarImagen();

  while(ros::ok())
  {
    //printf(". ");
    ros::spin();
  }

  printf("\n");
  exit(0);
}


