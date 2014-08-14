//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 30.3.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "../include/DriveableFloorCalib/qnode.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinectFloorCalib {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compute the sin/cos look-up tables (LUTs) for the v-disparity's
// Hough transform.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeVDLUTs(void)
{
  int i;
  double a, b;

// Start angle.
  a = 90.0;

// Loop through the angle range in 1/2 degree steps.
  for (i = 0; i < (2 * MAX_ANG); i++)
  {

// Compute the angle in radians from degrees.
    b = a * (M_PI / 180.0);
    VDCosLUT[i] = 2.0f * (float)cos(b);
    VDSinLUT[i] = 2.0f * (float)sin(b);
    a += 0.5;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compute a lower resolution disparity image from the input VGA
// resolution disparity image from a depth camera (Kinect or ASUS).
// Each pixel of the resulting disparity image corresponds to a 4x4
// pixel neighbourhood of the initial image.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeReducedResolutionDisparityImage(float *pDisp)
{
  unsigned int i, j, x, y, Xext, Xext4, Yext;
  float sum, cmp, res, a, *pTemp, *pResDisp;
  uint8_t cnt;

// Initialise the variables.
  Xext = ResultWidth;
  Xext4 = 4 * ResultWidth;
  Yext = ResultHeight;
  pResDisp = pResult;

// Loop through the input disparity image, four rows at a time.
  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {

// Search for the maximum disparity value of the 4x4 region.
      pTemp = pDisp;
      res = 0.0f;
      cnt = 0;
      for (j = 0; j < 4; j++)
      {
        for (i = 0; i < 4; i++)
        {
          a = pTemp[i];
          if ((a <= 99.0f) && (res < a)) res = a;
        }
        pTemp += Xext4;
      }

// If the maximum was not 0.0f, compute the mean of neighbouring disparity values.
      if (res != 0.0f)
      {
        pTemp = pDisp;
        cmp = res - 1.0f;
        sum = 0.0f;

// Take disparity values into account that are at most one disparity level lower than the maximum.
        for (j = 0; j < 4; j++)
        {
          for (i = 0; i < 4; i++)
          {
            a = pTemp[i];
            if (a > cmp) {sum += a; cnt++;}
          }
          pTemp += Xext4;
        }
        res = sum / (float)cnt;
      }
      pResDisp[x] = res;
      pDisp += 4;
    }

// Next four rows.
    pResDisp += Xext;
    pDisp += (12 * Xext);
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::Gradient(float Min, float Max)
{
  unsigned int x, y, Xext, Xext2, Yext;
  float a, *pI;
  uint8_t uc, *pMap;

  Xext = ResultWidth;
  Xext2 = 2 * Xext;
  Yext = ResultHeight;
  pI = pResult;
  pMap = pUseMap;

// Topmost row.
  for (x = 0; x < Xext; x++)
  {
    a = pI[x + Xext] - pI[x];
    uc = 1;
    if ((a < Min) || (a > Max)) uc = 0;
    pMap[x] = uc;
  }
  pMap += Xext;

// Rows between topmost and bottommost row.
  for (y = 1; y < (Yext - 1); y++)
  {
    for (x = 0; x < Xext; x++)
    {
      a = pI[x + Xext2] - pI[x];
      uc = 1;
      if ((a < Min) || (a > Max)) uc = 0;
      pMap[x] = uc;
    }
    pMap += Xext;
    pI += Xext;
  }

// Bottommost row.
  for (x = 0; x < Xext; x++)
  {
    a = pI[x + Xext] - pI[x];
    uc = 1;
    if ((a < Min) || (a > Max)) uc = 0;
    pMap[x] = uc;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method computes the v-disparity image for the reduced resolution
// disparity image.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeReducedVDisparity(float *pDisp)
{
  unsigned int x, y, Xext, Yext, index;
  uint16_t *pHist;
  uint8_t *pMap;
  float a;

  pHist = pHistAccu;
  pMap = pUseMap;
  Xext = ResultWidth;
  Yext = ResultHeight;

// Clear the histogram buffer.
  y = HIST_SIZE * ResultHeight;
  for (x = 0; x < y; x++) pHist[x] = 0;

// Let the pixels of the disparity image vote into the histogram buffer.
  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      index = (unsigned int)pDisp[x];
      if ((pMap[x] == 1) && (index < HIST_SIZE)) pHist[index]++;
    }
    pDisp += Xext;
    pMap += Xext;
    pHist += HIST_SIZE;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Find histogram bins that (1) are neighbouring, (2) have a high
// hit count and (3) only stretch over few (i.e. max 4) bins.
// Given the prerequisite that the robot is standing on a horizontal,
// planar and free-of-obstacles patch, such bin configurations must
// correspond to the floor.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::AnalyseHistogram(void)
{
  double a;
  unsigned int x, y, Xext, Yext, MaxIndex;
  uint16_t *pHist, u, MaxSum;

  Xext = HIST_SIZE / 2;
  Yext = ResultHeight;
  pHist = pHistAccu;

  for (y = 0; y < Yext; y++)
  {

// We don't want disparity '0' to vote; mark the result as initially invalid.
    pHist[0] = 0;
    MaxHistDisp[y] = 0.0f;
    MaxHistCnt[y] = 0;
    MaxSum = 0;

// Search for the maximum hit count within a three-bin wide sliding window. 
    for (x = 0; x < (Xext - 2); x++)
    {
      u = pHist[x] + pHist[x + 1] + pHist[x + 2];
      if (u > MaxSum)
      {
        MaxIndex = x;
        MaxSum = u;
      }
    }

// If there was a maximum of noteworthy value, store it.
    if (MaxSum > 32)
    {
      x = MaxIndex;
      a = ((double)pHist[x] * (double)x) +
          ((double)pHist[x + 1] * (double)(x + 1)) +
          ((double)pHist[x + 2] * (double)(x + 2));
      a /= (double)MaxSum;
      MaxHistDisp[y] = (float)a;
      MaxHistCnt[y] = u;
    }
    pHist += HIST_SIZE;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DoVDHoughTransform(float &k, float &d)
{
  int i, j, Index, AccuWidth, PointCount, s;
  uint16_t *pAccu, Max;
  float x, y;
  double r, a, lk, ld;

// Check if the desired start row is valid.
  k = 0.0f;
  d = 0.0f;

// Clear the Hough accu.
  pAccu = pVDHoughAccu;
  for (i = 0; i < (4 * HIST_SIZE * MAX_ANG); i++) pAccu[i] = 0;

// Compute the Hough transform.
  PointCount = 0;
  for (i = 0; i < (int)ResultHeight; i++)
  {
    pAccu = pVDHoughAccu;
    x = (float)i;
    y = MaxHistDisp[i];
    if (y > 0.0f)
    {
      PointCount++;
      for (j = 0; j < (2 * MAX_ANG); j++)
      {
        Index = (int)((x * VDCosLUT[j]) + (y * VDSinLUT[j]));
        if ((Index >= 0) && (Index < (2 * HIST_SIZE))) pAccu[Index]++;
        pAccu += (2 * HIST_SIZE);
      }
    }
  }

// Find the maximum.
  if (PointCount > 8)
  {
    pAccu = pVDHoughAccu;
    Max = 0;
    Index = 0;
    for (i = 0; i < (4 * HIST_SIZE * MAX_ANG); i++)
    {
      if (Max < pAccu[i])
      {
        Max = pAccu[i];
        Index = i;
      }
    }
    if (Max != 0)
    {
      i = Index % (2 * HIST_SIZE);
      j = Index / (2 * HIST_SIZE);

      if ((i > 0) && (i < ((2 * HIST_SIZE) - 1)) && (j > 0) && (j < ((2 * MAX_ANG) - 1)))
      {
        Index -= (2 * HIST_SIZE);
        Max = 3;
        PointCount = 0;
        r = a = 0.0;
        while (Max > 0)
        {
          s = (int)(pAccu[Index - 1] + pAccu[Index] + pAccu[Index + 1]);
          PointCount += s;
          r += (double)(((i - 1) * pAccu[Index - 1]) + (i * pAccu[Index]) + ((i + 1) * pAccu[Index + 1]));
          a += (double)((j - 1) * s);
          j++;
          Index += (2 * HIST_SIZE);
          Max--;
        }
        r = ((r / (double)PointCount) * 0.5);
        a = ((a / (double)PointCount) * 0.5);

        lk = tan(a * (M_PI / 180.0));
        ld = r * sqrt(1.0 + (lk * lk));
        k = (float)lk;
        d = (float)ld;
      }
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::Prune(float k, float d)
{
  float a;
  unsigned int u;

  for (u = 0; u < ResultHeight; u++)
  {
    a = (k * (float)u) + d;
    if ((a > 0.0f) && (MaxHistDisp[u] > 0.0f))
    {
      a = fabsf(a - MaxHistDisp[u]);
      if (a >= 1.0f) MaxHistDisp[u] = 0.0f;
    }
    else MaxHistDisp[u] = 0.0f;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::FitLine(void)
{
  double a[5], b, k, d;
  unsigned int x;

  k = d = 0.0f;
  a[4] = a[3] = a[2] = a[1] = a[0] = 0.0f;
  for (x = 0; x < ResultHeight; x++)
  {
    if (MaxHistDisp[x] > 0.0f)
    {
      a[0] += (double)(x * x);
      a[1] += (double)x;
      a[2] += 1.0f;
      a[3] += ((double)x * MaxHistDisp[x]);
      a[4] += MaxHistDisp[x];
    }
  }

  b = (a[0] * a[2]) - (a[1] * a[1]);
  if (b != 0.0f)
  {
    k = ((a[2] * a[3]) - (a[1] * a[4])) / b;
    if (a[1] != 0.0f) d = (a[3] - (k * a[0])) / a[1];
  }
  k_disp = (float)k;
  d_disp = (float)d;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback function that receives the disparity data.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& msg)
{
  float k, d;
  float *pInDisp;
  double duration;

  if (bRun)
  {
    ImgWidth = msg->image.width;
    ImgHeight = msg->image.height;
    ImgBytesPerRow = msg->image.step;
    pInDisp = (float *)(&(msg->image.data[0]));

//    duration = ros::Time::now().toSec();
    k_disp = 0.0f;
    d_disp = 0.0f;
    MakeReducedResolutionDisparityImage(pInDisp);
    Gradient(0.05f, 0.95f);
    MakeReducedVDisparity(pResult);
    AnalyseHistogram();
    DoVDHoughTransform(k, d);
    Prune(k, d);
    FitLine();
//    duration = ros::Time::now().toSec() - duration;
//    duration *= 1000000.0;
//    printf("- processing time: %iµs\n", (int)duration);

    FLOAT32_to_RGB888(pResult, pBuffer);
    VDisp_to_RGB888(pHistAccu, (pBuffer + (4 * ResultWidth)));
    Map_to_RGB888(pBuffer + (4 * ImgWidth * ResultHeight));
    VDHoughAccu_to_RGB888(pBuffer + (4 * ImgWidth * ResultHeight) + (4 * ResultWidth));

//    printf("y = %f * x + %f\n", k, d);

    Q_EMIT imageUpdated();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor of the class - allocated and configures resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
  printf("*** TopKinectFloorCalib constructor:\n");

  MakeVDLUTs();

  bRun = true;

// Default values that will be overwritten in the productive loop.
  ImgWidth = 640;
  ImgHeight = 480;

// Allocate the buffers for the result and local histogram.
  ResultWidth = ImgWidth / 4;
  ResultHeight = ImgHeight / 4;
  printf(" - result: %ux%u\n", ResultWidth, ResultHeight);

  printf(" - allocating (%u * %u) + %u float values\n", ResultWidth, ResultHeight, ResultHeight);
  pResult = new float[(3 * ResultWidth * ResultHeight) + ResultHeight];
  MaxHistDisp = pResult + (ResultWidth * ResultHeight);
  
  printf(" - allocating (%u * %u) + %u uint16_t values\n", HIST_SIZE, ResultHeight, ResultHeight);
  pHistAccu = new uint16_t[(HIST_SIZE * ResultHeight) + ResultHeight];
  MaxHistCnt = pHistAccu + (HIST_SIZE * ResultHeight);

  printf(" - allocating (%u * %u) uint8_t values\n", ResultWidth, ResultHeight);
  pUseMap = new uint8_t[ResultWidth * ResultHeight];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor of the class - waits for the ROS node to shut down and
// releases the allocated resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::~QNode()
{
  printf("*** TopKinectFloorCalib destructor:\n");

  bRun = false;
  sleep(1);

// Wait for the ROS node to shut down.
  printf(" - waiting for ROS to shut down\n");
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
  sleep(1);

// Release the buffers for the result and local histogram.
  if (pUseMap != 0)
  {
    printf(" - deleting uint8_t buffer\n");
    delete [] pUseMap;
    pUseMap = 0;
  }

  if (pHistAccu != 0)
  {
    printf(" - deleting uint16_t buffer\n");
    delete [] pHistAccu;
    pHistAccu = 0;
  }

  if (pResult != 0)
  {
    printf(" - deleting float buffer\n");
    delete [] pResult;
    pResult = 0;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Function that just starts the thread function.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool QNode::init()
{

// Start the thread function.
  printf("Starting subscriber thread...\n");
  start();
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Thread function - this is where the ROS main loop runs.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::run()
{

// Init the ROS stuff and create a subscriber.
  ros::init(init_argc, init_argv, "DriveableFloorCalib");
  ros::NodeHandle n("~");
  std::string disparity_img_topic;
  n.getParam ( "disparity_img_topic",  disparity_img_topic);

  if( disparity_img_topic.length() == 0 )
  {
      disparity_img_topic = "/camera/depth/disparity";
  }
  std::cout << "Subscribed to disparity image topic: " << disparity_img_topic << std::endl;

  ros::Subscriber sub2 = n.subscribe<stereo_msgs::DisparityImage>(disparity_img_topic, 1, &QNode::disparityCallback, this);

// ROS main loop.
  ros::spin();

// Send the shutdown signal.
  printf("Ros shutdown, proceeding to close the gui.\n");
  Q_EMIT rosShutdown();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::FLOAT32_to_RGB888(float *pIn, uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  float a, maxval;
  uint8_t uc;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);

  maxval = 0.0f;
  y = Xext * Yext;
  for (x = 0; x < y; x++)
    if (maxval < pIn[x]) maxval = pIn[x];
  if (maxval > 0.0f) maxval = 255.0f / maxval;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      a = pIn[x] * maxval;
      uc = (uint8_t)a;
      pOut[0] = uc; // B
      pOut[1] = uc; // G
      pOut[2] = uc; // R
      pOut[3] = 0xff; // A

      pOut += 4;
    }

    pIn += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::VDisp_to_RGB888(uint16_t *pIn, uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  uint8_t uc, *pAux8;
  uint16_t su;
  int i;

  Xext = HIST_SIZE;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);

  for (y = 0; y < Yext; y++)
  {
    pAux8 = pOut;
    for (x = 0; x < Xext; x++)
    {
      su = pIn[x];
      if (su > 255) su = 255;
      uc = (uint8_t)su;
      pOut[0] = uc; // B
      pOut[1] = uc; // G
      pOut[2] = uc; // R
      pOut[3] = 0xff; // A

      pOut += 4;
    }
/*
    i = 4 * (int)roundf(MaxHistDisp[y]);
    pAux8[i + 0] = 0x00; // B
    pAux8[i + 1] = 0xff; // G
    pAux8[i + 2] = 0x00; // R
*/
    i = (int)roundf((k_disp * (float)y) + d_disp);
    if (i < 0) i = 0;
    i *= 4;
    pAux8[i + 0] = 0x00; // B
    pAux8[i + 1] = 0x00; // G
    pAux8[i + 2] = 0xff; // R

    pIn += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::VDHoughAccu_to_RGB888(uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  uint8_t uc;
  uint16_t *pIn, Max;
  float a;

  Xext = (2 * HIST_SIZE);
  Yext = (2 * MAX_ANG);
  OutOffset = DispStep - (4 * Xext);
  pIn = pVDHoughAccu;

  y = Xext * Yext;
  Max = 0;
  for (x = 0; x < y; x++)
    if (Max < pIn[x]) Max = pIn[x];

  if (Max == 0) return;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      a = 255.0f * (float)pIn[x] / (float)Max;
      uc = (uint8_t)a;
      pOut[0] = uc; // B
      pOut[1] = uc; // G
      pOut[2] = uc; // R
      pOut[3] = 0xff; // A

      pOut += 4;
    }

    pIn += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::Map_to_RGB888(uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  uint8_t uc, *pIn;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);
  pIn = pUseMap;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      uc = pIn[x];
      if (uc == 1) uc = 255;
      pOut[0] = uc; // B
      pOut[1] = uc; // G
      pOut[2] = uc; // R
      pOut[3] = 0xff; // A
      pOut += 4;
    }

    pIn += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

