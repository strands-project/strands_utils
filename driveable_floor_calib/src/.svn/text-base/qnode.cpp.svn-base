//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 22.5.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "../include/TopKinect/qnode.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinect {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeLaserScanDXDY(void)
{
  int i, j;
  double Angle, dx, dy, x, y;

  for (i = 0; i < LASER_SCAN_SIZE; i++)
  {
    j = 0;
    x = 0.0;
    y = 0.0;
    Angle = (double)(90 - i) * (M_PI / 180.0);
    dx = cos(Angle);
    dy = sin(Angle);
    if (dx >= fabs(dy))
    {
      dy /= dx;
      dx = 1.0;
    }
    else
    {
      dx /= fabs(dy);
      if (dy > 0.0) dy = 1.0;
      else dy = -1.0;
    }

    while (1)
    {
      x += dx;
      y += dy;
      if ((x >= GRID_HEIGHT) || (fabs(y) > ((GRID_WIDTH - 1) / 2))) break;
      else j++;
    }

    LaserScanDX[i] = (float)dx;
    LaserScanDY[i] = (float)dy;
    LaserScanLength[i] = j;

printf("#%03i: length=%i | dx=%f | dy=%f\n", i, j, dx, dy);

  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Load the configuration file that provides the nominal v-disparity
// values corresponding to the floor as well as those when the robot
// is maximally tilted forward and backwards.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool QNode::LoadCfgFile(const char *FileName)
{
  char TextLine[32];
  const char *Params[8] = {"ImgW = ", "ImgH = ", "MinK = ", "MinD = ", "NomK = ", "NomD = ", "MaxK = ", "MaxD = "};
  float fData[8];
  int i;
  FILE *fp;
  bool bState = false;

// Open the file.
  printf("LoadCfgFile(): trying to open file '%s'\n", FileName);
  fp = fopen(FileName, "r");
  if (fp == 0)
  {
    printf(" - failed: '%s'\n\n", strerror(errno));
    return bState;
  }

// Read the file line by line.
  for (i = 0; i < 8; i++)
  {
    printf(" [%i] %s", (i + 1), Params[i]);
    if (fgets(TextLine, 32, fp) == 0)
    {
      printf("\n   - failed to read line #%i - ending\n", (i + 1));
      break;
    }
    else
    {
      if (strncmp((const char *)TextLine, Params[i], 6) != 0)
      {
        printf("\n   - expected parameter not found - ending\n");
        break;
      }
      fData[i] = (float)atof((const char *)(&TextLine[6]));
      printf("%f\n", fData[i]);
    }
  }
  if (i == 8) bState = true;

// Copy the parameters to the respective variables.
  if (bState)
  {
    ImgWidth = (unsigned int)fData[0];
    ImgHeight = (unsigned int)fData[1];
    MinK = fData[2];
    MinD = fData[3];
    NomK = fData[4];
    NomD = fData[5];
    MaxK = fData[6];
    MaxD = fData[7];
    printf(" - success\n");
  }

// Close the file.
  fclose(fp);
  return bState;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Based on the v-disparity limits read from the configuration file
// before, the corresponding limits for a Hough transform are deter-
// mined.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::GetVDHoughLimits(void)
{
  double r_min, r_max, a_min, a_max, a;

// Compute the distance and intercept of the min/max border lines.
  a_min = (180.0 / M_PI) * atan(MaxK);
  a_max = (180.0 / M_PI) * atan(MinK);
  r_min = MinD / sqrt(1.0 + (MinK * MinK));
  r_max = MaxD / sqrt(1.0 + (MaxK * MaxK));

  printf("*** Limits for v-disparity Hough:\n");
  printf("a_min = %f° | a_max = %f°\n", a_min, a_max);
  printf("r_min = %f | r_max = %f\n\n", r_min, r_max);

// Round to the next half disparity or angle step.
  a = floor(a_min);
  if ((a_min - a) >= 0.5) a += 0.5;
  else if (a_min == a) a -= 0.5;
  a_min = a;

  a = ceil(a_max);
  if ((a - a_max) > 0.5) a -= 0.5;
  else if (a == a_max) a += 0.5;
  a_max = a;

  a = floor(r_min);
  if ((r_min - a) >= 0.5) a += 0.5;
  else if (r_min == a) a -= 0.5;
  r_min = a;

  a = ceil(r_max);
  if ((a - r_max) > 0.5) a -= 0.5;
  else if (a == r_max) a += 0.5;
  r_max = a;

// Store the minimum distance/angle and the size of the Hough accu.
  VDMinAng = (float)a_min;
  VDMinDist = (float)r_min;
  VDHoughWidth = (int)(2.0 * (r_max - r_min)) + 1;
  VDHoughHeight = (int)(2.0 * (a_max - a_min)) + 1;

  printf("*** Adjusted values:\n");
  printf("a_min = %f° | a_max = %f°\n", a_min, a_max);
  printf("r_min = %f | r_max = %f\n\n", r_min, r_max);
  printf("v-disparity Hough accu dimensions are %ux%u\n\n", VDHoughWidth, VDHoughHeight);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Based on the v-disparity limits read from the configuration file
// before and on an assumed maximum roll angle of <see code>, the
// corresponding limits for a Hough transform of the *u-disparities*
// are determined.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::GetUDHoughLimits(void)
{
  double r_min, r_max, a_min, a_max, a;

// Compute the distance and intercept of the min/max border lines.
  a_min = -4.75;
  a_max = 4.5;
  r_min = MinD;
  r_max = MaxD;

  printf("*** Limits for u-disparity Hough:\n");
  printf("a_min = %f° | a_max = %f°\n", a_min, a_max);
  printf("r_min = %f | r_max = %f\n\n", r_min, r_max);

// Round to the next half disparity or angle step.
  a = floor(a_min);
  if ((a_min - a) >= 0.5) a += 0.5;
  else if (a_min == a) a -= 0.5;
  a_min = a;

  a = ceil(a_max);
  if ((a - a_max) > 0.5) a -= 0.5;
  else if (a == a_max) a += 0.5;
  a_max = a;

  a = floor(r_min);
  if ((r_min - a) >= 0.5) a += 0.5;
  else if (r_min == a) a -= 0.5;
  r_min = a;

  a = ceil(r_max);
  if ((a - r_max) > 0.5) a -= 0.5;
  else if (a == r_max) a += 0.5;
  r_max = a;

// Store the minimum distance/angle and the size of the Hough accu.
  UDMinAng = (float)a_min;
  UDHoughWidth = (int)(2.0 * (r_max - r_min)) + 1;
  UDHoughHeight = (int)(2.0 * (a_max - a_min)) + 1;

  printf("*** Adjusted values:\n");
  printf("a_min = %f° | a_max = %f°\n", a_min, a_max);
  printf("r_min = %f | r_max = %f\n\n", r_min, r_max);
  printf("u-disparity Hough accu dimensions are %ux%u\n\n", UDHoughWidth, UDHoughHeight);

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compute the sin/cos look-up tables (LUTs) for the v-disparity's
// Hough transform.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeVDLUTs(void)
{
  int i;
  double a, b;

  printf("*** Computing v-disparity sin/cos LUTs\n");

// Start angle.
  a = (double)VDMinAng + 90.0;

// Loop through the angle range in 1/2 degree steps.
  for (i = 0; i < VDHoughHeight; i++)
  {

// Compute the angle in radians from degrees.
    b = a * (M_PI / 180.0);
    VDCosLUT[i] = 2.0f * (float)cos(b);
    VDSinLUT[i] = 2.0f * (float)sin(b);
    a += 0.5;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Compute the sin/cos look-up tables (LUTs) for the u-disparity's
// Hough transform.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeUDLUTs(void)
{
  int i;
  double a, b;

  printf("*** Computing u-disparity sin/cos LUTs\n");

// Start angle.
  a = (double)UDMinAng + 90.0;

// Loop through the angle range in 1/2 degree steps.
  for (i = 0; i < UDHoughHeight; i++)
  {

// Compute the angle in radians from degrees.
    b = a * (M_PI / 180.0);
    UDCosLUT[i] = 2.0f * (float)cos(b);
    UDSinLUT[i] = 2.0f * (float)sin(b);
    a += 0.5;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method precalculates the minimum and maximum disparity values
// of two delimiting lines for each row of the reduced resolution
// disparity image.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeFloorDispLimits(void)
{
  unsigned int x;

  for (x = 0; x < ResultHeight; x++)
  {
    MinFloorDisp[x] = (MinK * (float)x) + MinD;
    MaxFloorDisp[x] = (MaxK * (float)x) + MaxD;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This method computes a reduced resolution (160x120) version of the
// initial VGA resolution (640x480) disparity image. Each resulting
// disparity value is this a local representative of a 4x4 pixel
// patch.
// For each such patch first the maximum disparity value is determined.
// If this value is not 0.0f, the mean of all values is computed that
// lie within a one disparity level neighbourhood of this maximum.
// The VGA resultion input image must be stored in the buffer indicated
// by <pDisp> and the reduced resolution disparity image will be stored
// in the buffer indicated by the member attribute <pResult[0]>.
// In addition, a "confidence" image is created whose uint8_t values
// (0..16) tell how many disparity values of the 4x4 region contributed
// to the resulting disparity value. This image is stored in the buffer
// indicated by the member attribute <pConfidence>.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeReducedResolutionDisparityImage(float *pDisp)
{
  unsigned int i, j, x, y, Xext, Xext4, Yext;
  float sum, cmp, res, a, *pTemp, *pResDisp;
  uint8_t *pResConf, cnt;

// Initialise the variables.
  Xext = ResultWidth;
  Xext4 = 4 * ResultWidth;
  Yext = ResultHeight;
  pResDisp = pResult[0];
  pResConf = pConfidence;

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
      pResConf[x] = cnt;

      pDisp += 4;
    }

// Next four rows.
    pResDisp += Xext;
    pResConf += Xext;
    pDisp += (12 * Xext);
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Creates a mask for the reduced resolution disparity image that tells
// which pixel is part of a valid 3x3 neighbourhood (1) and which is
// not (0).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::GetValid3x3RegionMask(void)
{
  int32_t min[3], *pIi;
  register int32_t a;
  unsigned int x, y, Xext, Xext2, Yext;
  uint8_t *pMask, uc;

  Xext = ResultWidth;
  Xext2 = 2 * Xext;
  Yext = ResultHeight;
  pIi = (int32_t *)pResult[0];
  pMask = pValid3x3;

// ****** Topmost row, leftmost column (4-pixel neighbourhood) ******
  a = pIi[0];
  if (a > pIi[Xext]) a = pIi[Xext];
  pIi++;
  min[1] = a;
  a = pIi[0];
  if (a > pIi[Xext]) a = pIi[Xext];
  pIi++;
  min[2] = a;
  if (a > min[1]) a = min[1];
  uc = 1;
  if (a == 0.0f) uc = 0;
  pMask[0] = uc;

// Columns between leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    min[0] = min[1];
    min[1] = min[2];
    a = pIi[0];
    if (a > pIi[Xext]) a = pIi[Xext];
    pIi++;
    min[2] = a;
    if (a > min[0]) a = min[0];
    if (a > min[1]) a = min[1];
    uc = 1;
    if (a == 0.0f) uc = 0;
    pMask[x] = uc;
  }

// Rightmost column (4-pixel neighbourhood).
  a = min[1];
  if (a > min[2]) a = min[2];
  uc = 1;
  if (a == 0.0f) uc = 0;
  pMask[x] = uc;
  pMask += Xext;
  pIi -= Xext;

// ****** Rows between the topmost and bottommost row ******
  for (y = 1; y < (Yext - 1); y++)
  {

// Leftmost column (6-pixel neighbourhood).
    a = pIi[0];
    if (a > pIi[Xext]) a = pIi[Xext];
    if (a > pIi[Xext2]) a = pIi[Xext2];
    pIi++;
    min[1] = a;
    a = pIi[0];
    if (a > pIi[Xext]) a = pIi[Xext];
    if (a > pIi[Xext2]) a = pIi[Xext2];
    pIi++;
    min[2] = a;
    if (a > min[1]) a = min[1];
    uc = 1;
    if (a == 0.0f) uc = 0;
    pMask[0] = uc;

// Columns between the leftmost and rightmost column (9-pixel neighbourhood).
    for (x = 1; x < (Xext - 1); x++)
    {
      min[0] = min[1];
      min[1] = min[2];
      a = pIi[0];
      if (a > pIi[Xext]) a = pIi[Xext];
      if (a > pIi[Xext2]) a = pIi[Xext2];
      pIi++;
      min[2] = a;
      if (a > min[0]) a = min[0];
      if (a > min[1]) a = min[1];
      uc = 1;
      if (a == 0.0f) uc = 0;
      pMask[x] = uc;
    }

// Rightmost column (6-pixel neighbourhood).
    a = min[1];
    if (a > min[2]) a = min[2];
    uc = 1;
    if (a == 0.0f) uc = 0;
    pMask[x] = uc;
    pMask += Xext;
  }

// ****** Bottommost row, leftmost column (4-pixel neighbourhood) ******
  a = pIi[0];
  if (a > pIi[Xext]) a = pIi[Xext];
  pIi++;
  min[1] = a;
  a = pIi[0];
  if (a > pIi[Xext]) a = pIi[Xext];
  pIi++;
  min[2] = a;
  if (a > min[1]) a = min[1];
  uc = 1;
  if (a == 0.0f) uc = 0;
  pMask[0] = uc;

// Columns between the leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    min[0] = min[1];
    min[1] = min[2];
    a = pIi[0];
    if (a > pIi[Xext]) a = pIi[Xext];
    pIi++;
    min[2] = a;
    if (a > min[0]) a = min[0];
    if (a > min[1]) a = min[1];
    uc = 1;
    if (a == 0.0f) uc = 0;
    pMask[x] = uc;
  }

// Rightmost column (4-pixel neighbourhood).
  a = min[1];
  if (a > min[2]) a = min[2];
  uc = 1;
  if (a == 0.0f) uc = 0;
  pMask[x] = uc;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Computes the v-disparity image from the reduced resolution dis-
// parity image. Furthermore, a label map is created: <0> means that
// the corresponding pixel of the disparity image was invalid, <1>
// means that disparity value was within the allowed minimum-maximum
// band and <3> means that the disparity value was outside the band.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeVDisparity(void)
{
  unsigned int x, y, Xext, Yext;
  int Index;
  float *pDisp, *pAccu;
  uint16_t *pHist, *pTotal, u;
  uint8_t *pLabel, Label;
  float dMin, dMax, a;

  pDisp = pResult[0];
  pHist = pHistAccu;
  pAccu = DispAccu;
  pTotal = TotalHistCnt;
  pLabel = pLabelMap;
  Xext = ResultWidth;
  Yext = ResultHeight;

// Clear the v-disparity buffer.
  y = HIST_SIZE * ResultHeight;
  for (x = 0; x < y; x++)
  {
    pHist[x] = 0;
    pAccu[x] = 0.0f;
  }

// Compute the v-disparity image row by row.
  for (y = 0; y < Yext; y++)
  {

// Precomputed minimum and maximum for pruning the input disparities.
    dMin = MinFloorDisp[y];
    dMax = MaxFloorDisp[y];
    u = 0;

    for (x = 0; x < Xext; x++)
    {
      Label = 1;
      a = pDisp[x];
      if ((a >= dMin) && (a <= dMax))
      {
        u++;
        Index = (int)a;
        pHist[Index]++;
        pAccu[Index] += a;
      }
      else if (a != 0.0) Label = 3;
      else Label = 0;
      pLabel[x] = Label;
    }
    pTotal[y] = u;

    pDisp += Xext;
    pHist += HIST_SIZE;
    pAccu += HIST_SIZE;
    pLabel += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeVDisparityFromLabelMap(void)
{
  unsigned x, y, Xext, Yext;
  int Index;
  float *pDisp, *pAccu;
  uint16_t *pHist, *pTotal, u;
  uint8_t *pLabel;
  float a;

  pDisp = pResult[0];
  pHist = pHistAccu;
  pAccu = DispAccu;
  pTotal = TotalHistCnt;
  pLabel = pLabelMap;
  Xext = ResultWidth;
  Yext = ResultHeight;

// Clear the v-disparity buffer.
  y = HIST_SIZE * ResultHeight;
  for (x = 0; x < y; x++)
  {
    pHist[x] = 0;
    pAccu[x] = 0.0f;
  }

// Compute the v-disparity image row by row.
  for (y = 0; y < Yext; y++)
  {
    u = 0;
    for (x = 0; x < Xext; x++)
    {
      if (pLabel[x] == 3)
      {
        a = pDisp[x];
        Index = (int)a;
        pHist[Index]++;
        pAccu[Index] += a;
        u++;
      }
    }
    pTotal[y] = u;

    pDisp += Xext;
    pHist += HIST_SIZE;
    pAccu += HIST_SIZE;
    pLabel += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::Binomial3x3(float *pI, float *pG)
{
  unsigned int x, y, Xext, Xext2, Yext;
  float s[3], a;
  uint8_t *pMask;

  Xext = ResultWidth;
  Xext2 = 2 * Xext;
  Yext = ResultHeight;
  pMask = pValid3x3;

// ****** Topmost row, leftmost column (4-pixel neighbourhood) ******
  s[1] = (2.0f * pI[0]) + pI[Xext];
  s[2] = (2.0f * pI[1]) + pI[Xext + 1];
  a = pI[0];
  pI += 2;
  if (pMask[0] != 0) a = (1.0f / 9.0f) * ((2.0f * s[1]) + s[2]);
  pG[0] = a;

// Columns between the leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    s[0] = s[1];
    s[1] = s[2];
    s[2] = (2.0f * pI[0]) + pI[Xext];
    a = pI[-1];
    pI++;
    if (pMask[x] != 0) a = (1.0f / 12.0f) * (s[0] + (2.0f * s[1]) + s[2]);
    pG[x] = a;
  }

// Rightmost column (4-pixel neighbourhood).
  a = pI[-1];
  if (pMask[x] != 0) a = (1.0f / 9.0f) * (s[1] + (2.0f * s[2]));
  pG[x] = a;
  pG += Xext;
  pMask += Xext;
  pI -= Xext;

// ****** Rows between the topmost and bottommost row ******
  for (y = 1; y < (Yext - 1); y++)
  {

// Leftmost column (6-pixel neighbourhood).
    s[1] = pI[0] + (2.0f * pI[Xext]) + pI[Xext2];
    s[2] = pI[1] + (2.0f * pI[Xext + 1]) + pI[Xext2 + 1];
    a = pI[Xext];
    pI += 2;
    if (pMask[0] != 0) a = (1.0f / 12.0f) * ((2.0f * s[1]) + s[2]);
    pG[0] = a;

// Columns between the leftmost and rightmost column (9-pixel neighbourhood).
    for (x = 1; x < (Xext - 1); x++)
    {
      s[0] = s[1];
      s[1] = s[2];
      s[2] = pI[0] + (2.0f * pI[Xext]) + pI[Xext2];
      a = pI[Xext - 1];
      pI++;
      if (pMask[x] != 0) a = (1.0f / 16.0f) * (s[0] + (2.0f * s[1]) + s[2]);
      pG[x] = a;
    }

// Rightmost column (6-pixel neighbourhood).
    a = pI[Xext - 1];
    if (pMask[x] != 0) a = (1.0f / 12.0f) * (s[1] + (2.0f * s[2]));
    pG[x] = a;
    pG += Xext;
    pMask += Xext;
  }

// ****** Bottom row, leftmost column (4-pixel neighbourhood) ******
  s[1] = pI[0] + (2.0f * pI[Xext]);
  s[2] = pI[1] + (2.0f * pI[Xext + 1]);
  a = pI[Xext];
  pI += 2;
  if (pMask[0] != 0) a = (1.0f / 9.0f) * ((2.0f * s[1]) + s[2]);
  pG[0] = a;

// Columns between the leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    s[0] = s[1];
    s[1] = s[2];
    s[2] = pI[0] + (2.0f * pI[Xext]);
    a = pI[Xext - 1];
    pI++;
    if (pMask[x] != 0) a = (1.0f / 12.0f) * (s[0] + (2.0f * s[1]) + s[2]);
    pG[x] = a;
  }

// Rightmost column (4-pixel neighbourhood).
  a = pI[Xext - 1];
  if (pMask[x] != 0) a = (1.0f / 9.0f) * (s[1] + (2.0f * s[2]));
  pG[x] = a;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::Gradient(float *pI, float *pG)
{
  unsigned int x, y, Xext, Xext2, Yext;
  float a, b, c, Bias;

  Xext = ResultWidth;
  Xext2 = 2 * Xext;
  Yext = ResultHeight;

// Topmost row.
  Bias = NomK;

  a = pI[0];
  b = pI[1];
  if ((a * b) != 0.0f) c = fabsf(a - b);
  else c = 0.0f;
  a = pI[0];
  b = pI[Xext];
  if ((a * b) != 0.0f)
  {
    a = fabsf(b - a - Bias);
    if (c < a) c = a;
  }
  pG[0] = c;

  for (x = 1; x < (Xext - 1); x++)
  {
    a = pI[0];
    b = pI[2];
    if ((a * b) != 0.0f) c = fabsf(a - b);
    else c = 0.0f;
    a = pI[1];
    b = pI[Xext + 1];
    if ((a * b) != 0.0f)
    {
      a = fabsf(b - a - Bias);
      if (c < a) c = a;
    }
    pG[x] = c;
    pI++;
  }

  a = pI[0];
  b = pI[1];
  if ((a * b) != 0.0f) c = fabsf(a - b);
  else c = 0.0f;
  a = pI[1];
  b = pI[Xext + 1];
  if ((a * b) != 0.0f)
  {
    a = fabsf(b - a - Bias);
    if (c < a) c = a;
  }
  pG[x] = c;
  pG += Xext;
  pI -= (Xext - 2);

// Rows between topmost and bottommost row.
  Bias = 2.0f * NomK;

  for (y = 1; y < (Yext - 1); y++)
  {
    a = pI[Xext];
    b = pI[Xext + 1];
    if ((a * b) != 0.0f) c = fabsf(a - b);
    else c = 0.0f;
    a = pI[0];
    b = pI[Xext2];
    if ((a * b) != 0.0f)
    {
      a = fabsf(b - a - Bias);
      if (c < a) c = a;
    }
    pG[0] = c;

    for (x = 1; x < (Xext - 1); x++)
    {
      a = pI[Xext];
      b = pI[Xext + 2];
      if ((a * b) != 0.0f) c = fabsf(a - b);
      else c = 0.0f;
      a = pI[1];
      b = pI[Xext2 + 1];
      if ((a * b) != 0.0f)
      {
        a = fabsf(b - a - Bias);
        if (c < a) c = a;
      }
      pG[x] = c;
      pI++;
    }

    a = pI[Xext];
    b = pI[Xext + 1];
    if ((a * b) != 0.0f) c = fabsf(a - b);
    else c = 0.0f;
    a = pI[1];
    b = pI[Xext2 + 1];
    if ((a * b) != 0.0f)
    {
      a = fabsf(b - a - Bias);
      if (c < a) c = a;
    }
    pG[x] = c;
    pG += Xext;
    pI += 2;
  }

// Bottommost row.
  Bias = NomK;

  a = pI[Xext];
  b = pI[Xext + 1];
  if ((a * b) != 0.0f) c = fabsf(a - b);
  else c = 0.0f;
  a = pI[0];
  b = pI[Xext];
  if ((a * b) != 0.0f)
  {
    a = fabsf(b - a - Bias);
    if (c < a) c = a;
  }
  pG[0] = c;

  for (x = 1; x < (Xext - 1); x++)
  {
    a = pI[Xext];
    b = pI[Xext + 2];
    if ((a * b) != 0.0f) c = fabsf(a - b);
    else c = 0.0f;
    a = pI[1];
    b = pI[Xext + 1];
    if ((a * b) != 0.0f)
    {
      a = fabsf(b - a - Bias);
      if (c < a) c = a;
    }
    pG[x] = c;
    pI++;
  }

  a = pI[Xext];
  b = pI[Xext + 1];
  if ((a * b) != 0.0f) c = fabsf(a - b);
  else c = 0.0f;
  a = pI[1];
  b = pI[Xext + 1];
  if ((a * b) != 0.0f)
  {
    a = fabsf(b - a - Bias);
    if (c < a) c = a;
  }
  pG[x] = c;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Computes a measure for the local smoothness [0.0..1.0].
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::LocalSmoothness(float *pI, float *pS)
{
  float mean[3], Mean, Std, a, dx, dy;
  unsigned int x, y, Xext, Xext2, Yext;
  uint8_t *pMask;

  Xext = ResultWidth;
  Xext2 = 2 * Xext;
  Yext = ResultHeight;
  pMask = pValid3x3;

// ****** Topmost row, leftmost column (quasi 9-pixel neighbourhood) ******
  mean[0] = (2.0f * pI[0]) + pI[Xext];
  mean[1] = mean[0];
  mean[2] = (2.0f * pI[1]) + pI[Xext + 1];

  a = 0.0f;
  if (pMask[0] != 0)
  {
    Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
    a = pI[0] - Mean;
    Std = (4.0f * (a * a));
    a = pI[1] - Mean;
    Std += (2.0f * (a * a));
    a = pI[Xext] - Mean;
    Std += (2.0f * (a * a));
    a = pI[Xext + 1] - Mean;
    Std += (a * a);
    if (Std == 0.0f) Std = 1.0f;

    dx = mean[2] - mean[0];
    dy = (2.0f * pI[Xext]) + pI[Xext + 1] - (2.0f * pI[0]) - pI[1];
    a = (dx * dx) + (dy * dy);
    a = (a * (1.0f / 6.0f)) / Std;
    if (a > 1.0f) a = 1.0f / a;
  }
  pS[0] = a;

// Columns between the leftmost and rightmost column (quasi 9-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    mean[0] = mean[1];
    mean[1] = mean[2];
    mean[2] = (2.0f * pI[2]) + pI[Xext + 2];

    a = 0.0f;
    if (pMask[x] != 0)
    {
      Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
      a = pI[0] - Mean;
      Std = (a * a);
      a = pI[1] - Mean;
      Std += (a * a);
      a = pI[2] - Mean;
      Std += (a * a);
      Std *= 2.0f;
      a = pI[Xext] - Mean;
      Std += (a * a);
      a = pI[Xext + 1] - Mean;
      Std += (a * a);
      a = pI[Xext + 2] - Mean;
      Std += (a * a);
      if (Std == 0.0f) Std = 1.0f;

      dx = mean[2] - mean[0];
      dy = pI[Xext] + pI[Xext + 1] + pI[Xext + 2] - pI[0] - pI[1] - pI[2];
      a = (dx * dx) + (dy * dy);
      a = (a * (1.0f / 6.0f)) / Std;
      if (a > 1.0f) a = 1.0f / a;
    }
    pS[x] = a;
    pI++;
  }

// Rightmost column (quasi 9-pixel neighbourhood).
  mean[0] = mean[1];
  mean[1] = mean[2];

  a = 0.0f;
  if (pMask[x] != 0)
  {
    Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
    a = pI[0] - Mean;
    Std = (2.0f * (a * a));
    a = pI[1] - Mean;
    Std += (4.0f * (a * a));
    a = pI[Xext] - Mean;
    Std += (a * a);
    a = pI[Xext + 1] - Mean;
    Std += (2.0f * (a * a));
    if (Std == 0.0f) Std = 1.0f;

    dx = mean[2] - mean[0];
    dy = pI[Xext] + (2.0f * pI[Xext + 1]) - pI[0] - (2.0f * pI[1]);
    a = (dx * dx) + (dy * dy);
    a = (a * (1.0f / 6.0f)) / Std;
    if (a > 1.0f) a = 1.0f / a;
  }
  pS[x] = a;
  pS += Xext;
  pMask += Xext;
  pI -= (Xext - 2);

// ****** Rows between the topmost and bottommost rows ******
  for (y = 1; y < (Yext - 1); y++)
  {

// Leftmost column (quasi 9-pixel neighbourhood).
    mean[0] = pI[0] + pI[Xext] + pI[Xext2];
    mean[1] = mean[0];
    mean[2] = pI[1] + pI[Xext + 1] + pI[Xext2 + 1];

    a = 0.0f;
    if (pMask[0] != 0)
    {
      Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
      a = pI[0] - Mean;
      Std = (2.0f * (a * a));
      a = pI[1] - Mean;
      Std += (a * a);
      a = pI[Xext] - Mean;
      Std += (2.0f * (a * a));
      a = pI[Xext + 1] - Mean;
      Std += (a * a);
      a = pI[Xext2] - Mean;
      Std += (2.0f * (a * a));
      a = pI[Xext2 + 1] - Mean;
      Std += (a * a);
      if (Std == 0.0f) Std = 1.0f;

      dx = mean[2] - mean[0];
      dy = (2.0f * pI[Xext2]) + pI[Xext2 + 1] - (2.0f * pI[0]) - pI[1];
      a = (dx * dx) + (dy * dy);
      a = (a * (1.0f / 6.0f)) / Std;
      if (a > 1.0f) a = 1.0f / a;
    }
    pS[0] = a;

// Columns between the leftmost and rightmost column (9-pixel neighbourhood).
    for (x = 1; x < (Xext - 1); x++)
    {
      mean[0] = mean[1];
      mean[1] = mean[2];
      mean[2] = pI[2] + pI[Xext + 2] + pI[Xext2 + 2];

      a = 0.0f;
      if (pMask[x] != 0)
      {
        Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
        a = pI[0] - Mean;
        Std = (a * a);
        a = pI[1] - Mean;
        Std += (a * a);
        a = pI[2] - Mean;
        Std += (a * a);
        a = pI[Xext] - Mean;
        Std += (a * a);
        a = pI[Xext + 1] - Mean;
        Std += (a * a);
        a = pI[Xext + 2] - Mean;
        Std += (a * a);
        a = pI[Xext2] - Mean;
        Std += (a * a);
        a = pI[Xext2 + 1] - Mean;
        Std += (a * a);
        a = pI[Xext2 + 2] - Mean;
        Std += (a * a);
        if (Std == 0.0f) Std = 1.0f;

        dx = mean[2] - mean[0];
        dy = pI[Xext2] + pI[Xext2 + 1] + pI[Xext2 + 2] - pI[0] - pI[1] - pI[2];
        a = (dx * dx) + (dy * dy);
        a = (a * (1.0f / 6.0f)) / Std;
        if (a > 1.0f) a = 1.0f / a;
      }
      pS[x] = a;
      pI++;
    }

// Rightmost column (quasi 9-pixel neighbourhood).
    mean[0] = mean[1];
    mean[1] = mean[2];

    a = 0.0f;
    if (pMask[x] != 0)
    {
      Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
      a = pI[0] - Mean;
      Std = (a * a);
      a = pI[1] - Mean;
      Std += (2.0f * (a * a));
      a = pI[Xext] - Mean;
      Std += (a * a);
      a = pI[Xext + 1] - Mean;
      Std += (2.0f * (a * a));
      a = pI[Xext2] - Mean;
      Std += (a * a);
      a = pI[Xext2 + 1] - Mean;
      Std += (2.0f * (a * a));
      if (Std == 0.0f) Std = 1.0f;

      dx = mean[2] - mean[0];
      dy = pI[Xext2] + (2.0f * pI[Xext2 + 1]) - pI[0] - (2.0f * pI[1]);
      a = (dx * dx) + (dy * dy);
      a = (a * (1.0f / 6.0f)) / Std;
      if (a > 1.0f) a = 1.0f / a;
    }
    pS[x] = a;
    pS += Xext;
    pMask += Xext;
    pI += 2;
  }

// ****** Bottommost row ******
  mean[0] = pI[0] + (2.0f * pI[Xext]);
  mean[1] = mean[0];
  mean[2] = pI[1] + (2.0f * pI[Xext + 1]);

  a = 0.0f;
  if (pMask[0] != 0)
  {
    Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
    a = pI[0] - Mean;
    Std = (2.0f * (a * a));
    a = pI[1] - Mean;
    Std += (a * a);
    a = pI[Xext] - Mean;
    Std += (4.0f * (a * a));
    a = pI[Xext + 1] - Mean;
    Std += (2.0f * (a * a));
    if (Std == 0.0f) Std = 1.0f;

    dx = mean[2] - mean[0];
    dy = (2.0f * pI[Xext]) + pI[Xext + 1] - (2.0f * pI[0]) - pI[1];
    a = (dx * dx) + (dy * dy);
    a = (a * (1.0f / 6.0f)) / Std;
    if (a > 1.0f) a = 1.0f / a;
  }
  pS[0] = a;

// Columns between the leftmost and rightmost column (quasi 9-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    mean[0] = mean[1];
    mean[1] = mean[2];
    mean[2] = pI[2] + (2.0f * pI[Xext + 2]);

    a = 0.0f;
    if (pMask[x] != 0)
    {
      Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
      a = pI[0] - Mean;
      Std = (a * a);
      a = pI[1] - Mean;
      Std += (a * a);
      a = pI[2] - Mean;
      Std += (a * a);
      a = pI[Xext] - Mean;
      Std += (2.0f * (a * a));
      a = pI[Xext + 1] - Mean;
      Std += (2.0f * (a * a));
      a = pI[Xext + 2] - Mean;
      Std += (2.0f * (a * a));
      if (Std == 0.0f) Std = 1.0f;

      dx = mean[2] - mean[0];
      dy = pI[Xext] + pI[Xext + 1] + pI[Xext + 2] - pI[0] - pI[1] - pI[2];
      a = (dx * dx) + (dy * dy);
      a = (a * (1.0f / 6.0f)) / Std;
      if (a > 1.0f) a = 1.0f / a;
    }
    pS[x] = a;
    pI++;
  }

// Rightmost column (quasi 9-pixel neighbourhood).
  mean[0] = mean[1];
  mean[1] = mean[2];

  a = 0.0f;
  if (pMask[x] != 0)
  {
    Mean = (1.0f / 9.0f) * (mean[0] + mean[1] + mean[2]);
    a = pI[0] - Mean;
    Std = (a * a);
    a = pI[1] - Mean;
    Std += (2.0f * (a * a));
    a = pI[Xext] - Mean;
    Std += (2.0f * (a * a));
    a = pI[Xext + 1] - Mean;
    Std += (4.0f * (a * a));
    if (Std == 0.0f) Std = 1.0f;

    dx = mean[2] - mean[0];
    dy = pI[Xext] + (2.0f * pI[Xext + 1]) - pI[0] - (2.0f * pI[1]);
    a = (dx * dx) + (dy * dy);
    a = (a * (1.0f / 6.0f)) / Std;
    if (a > 1.0f) a = 1.0f / a;
  }
  pS[x] = a;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// If no floor could be detected, this method is invoked to remove
// floor points from the reduced resolution disparity image.
// This is done by simply cutting out disparity values within a
// tolerance band around the *nominal* floor plane, denoted by the
// configuration parameters 'NomK' and NomD'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::Fallback(void)
{
  unsigned int x, y, Xext, Yext;
  float *pDisp, a, MinDisp, MaxDisp;
  uint8_t *pLabel;

  Xext = ResultWidth;
  Yext = ResultHeight;
  pDisp = pResult[0];
  pLabel = pLabelMap;

  for (y = 0; y < Yext; y++)
  {
    a = MaxHistDisp[y];
    if (a == 0.0f) a = (NomK * (float)y) + NomD;
    MinDisp = a - 2.0f;
    MaxDisp = a + 2.0f;
    for (x = 0; x < ResultWidth; x++)
    {
      a = pDisp[x];
      if ((a >= MinDisp) && (a <= MaxDisp))
      {
        pDisp[x] = 0.0f;
        pLabel[x] = 0;
      }
    }
    pDisp += Xext;
    pLabel += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Given the angle (Gamma) by which the camera is tilted downwards,
// there is a grid where the center of the bottommost row, mid cell
// is directly located underneath the optical center of the camera.
// The grid extends symmetrically left and right, but only un forward
// direction. If the camera was tilted downwards so much that it sees
// points behind its projected center, it wouldn't be the bottommost
// row, but higher up.
// The transform into the robot coordinate system is done later and
// elsewhere via tf.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::FillObstacleGrid(void)
{
  unsigned int Index, LoopCtr;
  uint16_t *pGrid;
  float *pX, *pY, *pZ, CosGamma, SinGamma, Scale;
  uint8_t *pLabel;
  int xr, yr;

// Initialise the obstacle grid.
  pGrid = pObstacleGrid;
  LoopCtr = GRID_WIDTH * GRID_HEIGHT;
  for (Index = 0; Index < LoopCtr; Index++) pGrid[Index] = 0;

// Fill the grid.
  CosGamma = COS_PLANE_GAMMA;
  SinGamma = SIN_PLANE_GAMMA;
  Scale = 1.0f / GRID_RESOLUTION;
  pX = pResult[0];
  pY = pResult[1];
  pZ = pResult[2];
  pLabel = pLabelMap;
  LoopCtr = ResultWidth * ResultHeight;
  for (Index = 0; Index < LoopCtr; Index++)
  {
    if ((pLabel[Index] == 1) || (pLabel[Index] == 2))
    {
      xr = (int)(((pZ[Index] * CosGamma) - (pY[Index] * SinGamma)) * Scale);
      yr = (int)((GRID_WIDTH / 2.0f) - (pX[Index] * Scale));
      if ((xr >= 0) && (xr < GRID_HEIGHT))
      {
        if ((yr >= 0) && (yr < GRID_WIDTH))
        {
          yr += (xr * GRID_WIDTH);
          pGrid[yr]++;
        }
      }
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::FilterObstacleGrid(void)
{
  unsigned int x, y, Xext, Xext2, Yext;
  uint16_t s[3], *pI, *pG, a, Thrsh;

  Xext = GRID_WIDTH;
  Xext2 = 2 * Xext;
  Yext = GRID_HEIGHT;
  pI = pObstacleGrid;
  pG = pObstacleGrid2;
  Thrsh = GridThreshold;

// ****** Topmost row, leftmost column (4-pixel neighbourhood) ******
  s[1] = pI[0] + pI[Xext];
  s[2] = pI[1] + pI[Xext + 1];
  a = pI[0];
  pI += 2;
  if ((s[1] + s[2]) < Thrsh) a = 0;
  pG[0] = a;

// Columns between the leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    s[0] = s[1];
    s[1] = s[2];
    s[2] = pI[0] + pI[Xext];
    a = pI[-1];
    pI++;
    if ((s[0] + s[1] + s[2]) < Thrsh) a = 0;
    pG[x] = a;
  }

// Rightmost column (4-pixel neighbourhood).
  a = pI[-1];
  if ((s[1] + s[2]) < Thrsh) a = 0;
  pG[x] = a;
  pG += Xext;
  pI -= Xext;

// ****** Rows between the topmost and bottommost row ******
  for (y = 1; y < (Yext - 1); y++)
  {

// Leftmost column (6-pixel neighbourhood).
    s[1] = pI[0] + pI[Xext] + pI[Xext2];
    s[2] = pI[1] + pI[Xext + 1] + pI[Xext2 + 1];
    a = pI[Xext];
    pI += 2;
    if ((s[1] + s[2]) < Thrsh) a = 0;
    pG[0] = a;

// Columns between the leftmost and rightmost column (9-pixel neighbourhood).
    for (x = 1; x < (Xext - 1); x++)
    {
      s[0] = s[1];
      s[1] = s[2];
      s[2] = pI[0] + pI[Xext] + pI[Xext2];
      a = pI[Xext - 1];
      pI++;
      if ((s[0] + s[1] + s[2]) < Thrsh) a = 0;
      pG[x] = a;
    }

// Rightmost column (6-pixel neighbourhood).
    a = pI[Xext - 1];
    if ((s[1] + s[2]) < Thrsh) a = 0;
    pG[x] = a;
    pG += Xext;
  }

// ****** Bottom row, leftmost column (4-pixel neighbourhood) ******
  s[1] = pI[0] + pI[Xext];
  s[2] = pI[1] + pI[Xext + 1];
  a = pI[Xext];
  pI += 2;
  if ((s[1] + s[2]) < Thrsh) a = 0;
  pG[0] = a;

// Columns between the leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    s[0] = s[1];
    s[1] = s[2];
    s[2] = pI[0] + pI[Xext];
    a = pI[Xext - 1];
    pI++;
    if ((s[0] + s[1] + s[2]) < Thrsh) a = 0;
    pG[x] = a;
  }

// Rightmost column (4-pixel neighbourhood).
  a = pI[Xext - 1];
  if ((s[1] + s[2]) < Thrsh) a = 0;
  pG[x] = a;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// FIXME: This is just a quickhack test for masking out the platform.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MaskPlatformFromObstacleGrid(unsigned int Selector)
{
  unsigned int x, y, BlankWidth, BlankHeight, BlankStart;
  uint16_t *pGrid;

  if (Selector == 0) pGrid = pObstacleGrid;
  else pGrid = pObstacleGrid2;

/* Values for PT1-1 */
/*  BlankStart = 33;
  BlankWidth = 30;
  BlankHeight = 14;*/

/* New values for PT1-1 */
  BlankStart = 33;
  BlankWidth = 50;
  BlankHeight = 20;

/* Values for PT1-2
  BlankStart = 34;
  BlankWidth = 27;
  BlankHeight = 9; */

  pGrid += BlankStart;

  //for setting the blanking rectangle
/*  for (y = 0; y < (BlankHeight - 1); y++)
  {
    pGrid[0] = 1;
    pGrid[BlankWidth - 1] = 1;
    pGrid += GRID_WIDTH;
  }
  for (x = 0; x < BlankWidth; x++) pGrid[x] = 1; */


/* for blanking out the platform from the occupancy grid */
  for (y = 0; y < BlankHeight; y++)
  {
    for (x = 0; x < BlankWidth; x++) pGrid[x] = 0;
    pGrid += GRID_WIDTH;
  }

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Computes a virtual floor-parallel 2D laser scan from the obstacle
// occupancy grid.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::LaserScanFromObstacleGrid(void)
{
  int i, j, k;
  int IndexX, IndexY, IncrementI, GridIndex;
  float a, IncrementF;
  uint16_t *pGrid;

// Get the pointer to the occupancy grid;
  pGrid = pObstacleGrid2;

// Mark all laser scan readings as out of range.
  for (i = 0; i < LASER_SCAN_SIZE; i++) obstacle_scan.ranges[i] = LASER_SCAN_OUT_OF_RANGE;

// Perform raytracing for each beam of the virtual laser.
  for (i = 0; i < LASER_SCAN_SIZE; i++)
  {

// Get the number of cells to be checked for the current virtual laser beam.
    k = LaserScanLength[i];

// If the virtual laser beam is mainly oriented forward...
    if (LaserScanDX[i] == 1.0f)
    {

// ...initialise indices and index increment for forward scanning.
      IndexX = 0;
      a = (float)((GRID_WIDTH - 1) / 2);
      IncrementF = LaserScanDY[i];

// Scan the cells along the virtual laser beam.
      for (j = 1; j <= k; j++)
      {

// Get the index into the occupancy grid for the current grid cell.
        IndexX++;
        a += IncrementF;
        IndexY = (int)a;
        GridIndex = (IndexX * GRID_WIDTH) + IndexY;

// If an occupied grid cell is found...
        if (pGrid[GridIndex] != 0)
        {

// ...calculate the length of the virtual laser beam, store it and end the raytracing for this beam.
          a -= (float)((GRID_WIDTH - 1) / 2);
          a = sqrtf((float)(j * j) + (a * a));
          obstacle_scan.ranges[LASER_SCAN_SIZE - 1 - i] = a * GRID_RESOLUTION;
          break;
        }
        else pObstacleGrid[GridIndex] = 1; // only for visualising the raytracing
      }
    }

// If the virtual laser beam is mainly oriented sidewards...
    else
    {

// ...initialise indices and index increment for sidewards scanning.
      a = 0.0f;
      IndexY = (GRID_WIDTH - 1) / 2;
      IncrementF = LaserScanDX[i];
      IncrementI = (int)LaserScanDY[i];

// Scan the cells along the virtual laser beam.
      for (j = 1; j <= k; j++)
      {

// Get the index into the occupancy grid for the current grid cell.
        a += IncrementF;
        IndexX = (int)a;
        IndexY += IncrementI;
        GridIndex = (IndexX * GRID_WIDTH) + IndexY;

// If an occupied grid cell is found...
        if (pGrid[GridIndex] != 0)
        {

// ...calculate the length of the virtual laser beam, store it and end the raytracing for this beam.
          a = sqrtf((float)(j * j) + (a * a));
          obstacle_scan.ranges[LASER_SCAN_SIZE - 1 - i] = a * GRID_RESOLUTION;
          break;
        }
        else pObstacleGrid[GridIndex] = 1; // only for visualising the raytracing
      }
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::FindRelevantHistogramBins(void)
{
  unsigned int x, y, Xext, Yext, MaxIndex;
  uint16_t *pHist, MaxCnt, u;
  double a;
  float *pAccu;

  Xext = HIST_SIZE;
  Yext = ResultHeight;
  pHist = pHistAccu;
  pAccu = DispAccu;

// Loop through all rows of the v-disparity image.
  for (y = 0; y < Yext; y++)
  {
    a = 0.0;
    MaxCnt = 0;
    for (x = 0; x < (Xext - 1); x++)
    {
      u = pHist[x] + pHist[x + 1];
      if (MaxCnt < u)
      {
        MaxCnt = u;
        MaxIndex = x;
      }
    }

    if (MaxCnt > RELEVANT_HIT_MIN)
    {
      x = MaxIndex;
      a = (double)pAccu[x] + (double)pAccu[x + 1];
      a /= (double)MaxCnt;
    }

    MaxHistDisp[y] = (float)a;
    MaxHistCnt[y] = MaxCnt;

    pHist += HIST_SIZE;
    pAccu += HIST_SIZE;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MakeInitialLabelMap(void)
{
  unsigned int x, y, Xext, Yext;
  float *pDisp, *pGrad, a, MinDisp, MaxDisp, Thrsh;
  uint8_t *pLabel, uc;

  Xext = ResultWidth;
  Yext = ResultHeight;
  pLabel = pLabelMap;
  pDisp = pResult[0];
  pGrad = pResult[3];
  Thrsh = GradientThreshold;

  for (y = 0; y < Yext; y++)
  {
    MinDisp = MinFloorDisp[y];
    MaxDisp = MaxFloorDisp[y];
    for (x = 0; x < Xext; x++)
    {
      uc = 0;
      a = pDisp[x];
      if (a != 0.0f)
      {
        uc++;
        if ((a >= MinDisp) && (a <= MaxDisp))
        {
          uc++;
          if (pGrad[x] < Thrsh) uc++;
        }
      }
      pLabel[x] = uc;
    }
    pDisp += Xext;
    pGrad += Xext;
    pLabel += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::RemoveFloorCheap(float k, float d)
{
  unsigned int x, y, Xext, Yext;
  float a, *pDisp, dMin, dMax, tolerance;

  Xext = ResultWidth;
  Yext = ResultHeight;
  pDisp = pResult[0];

  for (y = 0; y < Yext; y++)
  {
    dMin = (k * (float)y) + d;
    tolerance = 1.0f + ((float)y / (float)ResultHeight);
    dMax = dMin + tolerance;
    dMin -= tolerance;

    for (x = 0; x < Xext; x++)
    {
      a = pDisp[x];
      if ((a >= dMin) && (a <= dMax)) a = 0.0f;
      pDisp[x] = a;
    }
    pDisp += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::MarkFloorCandidates(void)
{
  unsigned int x, y, Xext, Yext;
  float *pDisp, lDisp;
  uint8_t *pLabel;

  Xext = ResultWidth;
  Yext = ResultHeight;
  pDisp = pResult[0];
  pLabel = pLabelMap;

  for (y = 0; y < Yext; y++)
  {
    lDisp = MaxHistDisp[y];

    if (lDisp != 0.0f)
    {
      for (x = 0; x < Xext; x++)
        if (pLabel[x] != 0)
          if (fabsf(pDisp[x] - lDisp) <= 0.5f) pLabel[x] = 4;
    }

    pDisp += Xext;
    pLabel += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::RemoveGhosts(void)
{
  unsigned int x, y, Xext, Yext;
  float *pDisp, *pGrad, lDisp, a;
  uint8_t *pLabel;

  Xext = ResultWidth;
  Yext = ResultHeight;
  pDisp = pResult[0];
  pGrad = pResult[3];
  pLabel = pLabelMap;

// Loop through all rows of the maxima of the v-disparity image.
  for (y = 0; y < Yext; y++)
  {

// Get the maximum of the current row and check if it is valid - if not, fall back to static calibration.
    lDisp = MaxHistDisp[y];
    if (lDisp == 0.0f) lDisp = (NomK * (float)y) + NomD;

    for (x = 0; x < Xext; x++)
    {
      if (pLabel[x] == 2)
      {
        a = 2.0f * fabsf(pDisp[x] - lDisp);
        if (a < pGrad[x]) pLabel[x] = 3;
      }
    }

    pDisp += Xext;
    pGrad += Xext;
    pLabel += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DoVDHoughTransform(float &k, float &d, int StartRow)
{
  int i, j, LoopCtr, Index, AccuWidth, PointCount, s;
  uint16_t *pAccu, Max;
  float x, y, Min;
  double r, a, lk, ld;

// Check if the desired start row is valid.
  k = 0.0f;
  d = 0.0f;
  if (StartRow >= (int)ResultHeight) return;

// Clear the Hough accu.
  pAccu = pVDHoughAccu;
  LoopCtr =  VDHoughWidth * VDHoughHeight;
  for (i = 0; i < LoopCtr; i++) pAccu[i] = 0;

// Compute the Hough transform.
  PointCount = 0;
  LoopCtr = VDHoughHeight;
  AccuWidth = VDHoughWidth;
  Min = 2.0f * VDMinDist;
  for (i = StartRow; i < (int)ResultHeight; i++)
  {
    pAccu = pVDHoughAccu;
    x = (float)i;
    y = MaxHistDisp[i];
    if (y > 0.0f)
    {
      PointCount++;
      for (j = 0; j < LoopCtr; j++)
      {
        Index = (int)((x * VDCosLUT[j]) + (y * VDSinLUT[j]) - Min);
        if ((Index >= 0) && (Index < AccuWidth)) pAccu[Index]++;
        pAccu += AccuWidth;
      }
    }
  }

// Find the maximum.
  if (PointCount > 8)
  {
    pAccu = pVDHoughAccu;
    LoopCtr =  VDHoughWidth * VDHoughHeight;
    Max = 0;
    Index = 0;
    for (i = 0; i < LoopCtr; i++)
    {
      if (Max < pAccu[i])
      {
        Max = pAccu[i];
        Index = i;
      }
    }
    if (Max != 0)
    {
      i = Index % VDHoughWidth;
      j = Index / VDHoughWidth;

      if ((i > 0) && (i < (VDHoughWidth - 1)) && (j > 0) && (j < (VDHoughHeight - 1)))
      {
        Index -= VDHoughWidth;
        LoopCtr = 3;
        PointCount = 0;
        r = a = 0.0;
        while (LoopCtr > 0)
        {
          s = (int)(pAccu[Index - 1] + pAccu[Index] + pAccu[Index + 1]);
          PointCount += s;
          r += (double)(((i - 1) * pAccu[Index - 1]) + (i * pAccu[Index]) + ((i + 1) * pAccu[Index + 1]));
          a += (double)((j - 1) * s);
          j++;
          Index += VDHoughWidth;
          LoopCtr--;
        }
        r = ((r / (double)PointCount) * 0.5) + (double)VDMinDist;
        a = ((a / (double)PointCount) * 0.5) + (double)VDMinAng;

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
void QNode::DoVDFitLine(float &k, float &d)
{
  float a[5], b, y;
  unsigned int x;

  a[4] = a[3] = a[2] = a[1] = a[0] = 0.0f;
  for (x = 0; x < ResultHeight; x++)
  {
    y = (k * x) + d;
    if (fabsf(MaxHistDisp[x] - y) <= 1.0f)
    {
      a[0] += (float)(x * x);
      a[1] += (float)x;
      a[2] += 1.0f;
      a[3] += ((float)x * MaxHistDisp[x]);
      a[4] += MaxHistDisp[x];
    }
  }

  b = (a[0] * a[2]) - (a[1] * a[1]);
  if (b != 0.0f)
  {
    k = ((a[2] * a[3]) - (a[1] * a[4])) / b;
    if (a[1] != 0.0f) d = (a[3] - (k * a[0])) / a[1];
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DoUDHoughTransform(float &k, float &d, int &PointCount, int &MaxCount, unsigned int RowNumber)
{
  int i, j, LoopCtr, AccuWidth, HorOffset, Index, s;
  uint16_t *pAccu;
  uint8_t *pLabel;
  float *pDisp, x, y, Min, Max, SubMin;
  double r, a, lk, ld;

  PointCount = 0;
  MaxCount = 0;

// Get the pointer to the desired row of the reduced resolution disparity image.
  if (RowNumber >= ResultHeight) return;
  pDisp = pResult[0] + (RowNumber * ResultWidth);
  HorOffset = (int)(ResultWidth / 2);

// Get the horizontal limits of the Hough accu.
  Min = MinFloorDisp[RowNumber];
  x = floor(Min);
  if ((Min - x) >= 0.5) x += 0.5;
  else if (x == Min) x -= 0.5;

  Max = MaxFloorDisp[RowNumber];
  y = ceil(Max);
  if ((y - Max) > 0.5) y -= 0.5;
  else if (y == Max) y += 0.5;

  AccuWidth = (int)(2.0 * (y - x)) + 1;
  SubMin = 2.0f * x;

// Clear the Hough accu.
  pAccu = pUDHoughAccu;
  LoopCtr =  AccuWidth * UDHoughHeight;
  for (i = 0; i < LoopCtr; i++) pAccu[i] = 0;

// Compute the Hough transform.
  pLabel = pLabelMap;
  LoopCtr = UDHoughHeight;
  for (i = 0; i < (int)ResultWidth; i++)
  {
    if (pLabel[i] == 1)
    {
      pAccu = pUDHoughAccu;
      x = (float)(i - HorOffset);
      y = pDisp[i];
      PointCount++;
      for (j = 0; j < LoopCtr; j++)
      {
        Index = (int)((x * UDCosLUT[j]) + (y * UDSinLUT[j]) - SubMin);
        if ((Index >= 0) && (Index < AccuWidth)) pAccu[Index]++;
        pAccu += AccuWidth;
      }
    }
  }

// Find the maximum.
  if (PointCount > 8)
  {
    pAccu = pUDHoughAccu;
    LoopCtr = AccuWidth * UDHoughHeight;
    Index = 0;
    for (i = 0; i < LoopCtr; i++)
    {
      if (MaxCount < pAccu[i])
      {
        MaxCount = pAccu[i];
        Index = i;
      }
    }
    if (MaxCount != 0)
    {
      i = Index % AccuWidth;
      j = Index / AccuWidth;

      if ((i > 0) && (i < (AccuWidth - 1)) && (j > 0) && (j < (UDHoughHeight - 1)))
      {
        Index -= AccuWidth;
        LoopCtr = 3;
        PointCount = 0;
        r = a = 0.0;
        while (LoopCtr > 0)
        {
          s = (int)(pAccu[Index - 1] + pAccu[Index] + pAccu[Index + 1]);
          PointCount += s;
          r += (double)(((i - 1) * pAccu[Index - 1]) + (i * pAccu[Index]) + ((i + 1) * pAccu[Index + 1]));
          a += (double)((j - 1) * s);
          j++;
          Index += AccuWidth;
          LoopCtr--;
        }
        r = 0.5 * ((r / (double)PointCount) + (double)SubMin);
        a = ((a / (double)PointCount) * 0.5) + (double)UDMinAng;

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
void QNode::DoUDFloorLabel(float k, float d, unsigned int RowNumber)
{
  float x, y, HorOffset, *pDisp;
  uint8_t *pLabel;
  int i;

  pLabel = pLabelMap + (ResultWidth * RowNumber);
  pDisp = pResult[0] + (ResultWidth * RowNumber);
  HorOffset = (float)(ResultWidth / 2);

  for (i = 0; i < (int)ResultWidth; i++)
  {
    if (pLabel[i] == 1)
    {
      x = (float)i - HorOffset;
      y = (k * x) + d;
      if (fabsf(pDisp[i] - y) <= 1.0f) pLabel[i] = 2;
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DoUDFitLine(float &k, float &d, unsigned int RowNumber)
{
  float a[5], b, x, y, HorOffset, *pDisp;
  int i;

  pDisp = pResult[0] + (ResultWidth * RowNumber);
  HorOffset = (float)(ResultWidth / 2);
  a[4] = a[3] = a[2] = a[1] = a[0] = 0.0f;
  for (i = 0; i < (int)ResultWidth; i++)
  {
    x = (float)i - HorOffset;
    y = (k * x) + d;
    if (fabsf(pDisp[i] - y) <= 1.0f)
    {
      a[0] += x * x;
      a[1] += x;
      a[2] += 1.0f;
      a[3] += (x * pDisp[i]);
      a[4] += pDisp[i];
    }
  }

  b = (a[0] * a[2]) - (a[1] * a[1]);
  if (b != 0.0f)
  {
    k = ((a[2] * a[3]) - (a[1] * a[4])) / b;
    if (a[1] != 0.0f) d = (a[3] - (k * a[0])) / a[1];
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DisparityTo3D(float *pDisp)
{
  unsigned int x, y, Xext, Yext, Index;
  float cx, cy, u, v, f, T, z;
  float *pX, *pY, *pZ;

  Xext = ResultWidth;
  Yext = ResultHeight;

// Camera parameters for 3D projection.
  cx = P_cx;
  cy = P_cy;
  f = FocalLength;
  T = Baseline;

// Get the pointers to the 3D data buffers. The buffer pX initially holds the disparity values.
  pX = pResult[0];
  pY = pResult[1];
  pZ = pResult[2];

  Index = 0;
  v = 1.5f;
  for (y = 0; y < Yext; y++)
  {
    u = 1.5f;
    for (x = 0; x < Xext; x++)
    {

// Get the disparity value - invalid values are 0.0f;
      z = pDisp[Index];
      if (z != 0.0f) z = T / z;

// Compute the 3D data.
      pX[Index] = z * (u - cx);
      pY[Index] = z * (v - cy);
      pZ[Index] = z * f;
      Index++;
      u += 4.0f;
    }
    v += 4.0f;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Computes the surface normals from the 3D data.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::SurfaceNormals(void)
{
  float *pX, *pY, *pZ, *pNx, *pNy, *pNz;
  float ax, ay, az, bx, by, bz, nx, ny, nz;
  unsigned int x, y, Xext, Xext2, Yext;
  uint8_t *pValid;

// Get the width and height of the image.
  Xext = ResultWidth;
  Xext2 = 2 * Xext;
  Yext = ResultHeight;

// Get the pointers to the coordinates of the 3D data.
  pX = pResult[0];
  pY = pResult[1];
  pZ = pResult[2];

// Get the pointers to the components of the surface normals.
  pNx = pResult[3];
  pNy = pResult[4];
  pNz = pResult[5];

// Get the pointer to the mask of valid 3x3 neighbourhoods.
  pValid = pValid3x3;

// ****** Topmost row, leftmost column (4-pixel neighbourhood) ******
  if (pValid[0] != 0)
  {
    ax = pX[1] - pX[0] + pX[Xext + 1] - pX[Xext];
    ay = pY[1] - pY[0] + pY[Xext + 1] - pY[Xext];
    az = pZ[1] - pZ[0] + pZ[Xext + 1] - pZ[Xext];
    bx = pX[0] + pX[1] - pX[Xext] - pX[Xext + 1];
    by = pY[0] + pY[1] - pY[Xext] - pY[Xext + 1];
    bz = pZ[0] + pZ[1] - pZ[Xext] - pZ[Xext + 1];
    nx = (ay * bz) - (az * by);
    ny = (az * bx) - (ax * bz);
    nz = (ax * by) - (ay * bx);
    if (ax == 0.0f) ax = 1.0f;
    ax = 1.0f / sqrtf(ax);
    pNx[0] = nx * ax;
    pNy[0] = ny * ax;
    pNz[0] = nz * ax;
  }
  else
  {
    pNx[0] = 0.0f;
    pNy[0] = 0.0f;
    pNz[0] = 0.0f;
  }

// Columns between the leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    if (pValid[x] != 0)
    {
      ax = pX[2] - pX[0] + pX[Xext + 2] - pX[Xext];
      ay = pY[2] - pY[0] + pY[Xext + 2] - pY[Xext];
      az = pZ[2] - pZ[0] + pZ[Xext + 2] - pZ[Xext];
      bx = pX[0] + pX[1] + pX[2] - pX[Xext] - pX[Xext + 1] - pX[Xext + 2];
      by = pY[0] + pY[1] + pY[2] - pY[Xext] - pY[Xext + 1] - pY[Xext + 2];
      bz = pZ[0] + pZ[1] + pZ[2] - pZ[Xext] - pZ[Xext + 1] - pZ[Xext + 2];
      nx = (ay * bz) - (az * by);
      ny = (az * bx) - (ax * bz);
      nz = (ax * by) - (ay * bx);
      ax = (nx * nx) + (ny * ny) + (nz * nz);
      if (ax == 0.0f) ax = 1.0f;
      ax = 1.0f / sqrtf(ax);
      pNx[x] = nx * ax;
      pNy[x] = ny * ax;
      pNz[x] = nz * ax;
    }
    else
    {
      pNx[x] = 0.0f;
      pNy[x] = 0.0f;
      pNz[x] = 0.0f;
    }
    pX++;
    pY++;
    pZ++;
  }

// Rightmost column (4-pixel neighbourhood).
  if (pValid[x] != 0)
  {
    ax = pX[1] - pX[0] + pX[Xext + 1] - pX[Xext];
    ay = pY[1] - pY[0] + pY[Xext + 1] - pY[Xext];
    az = pZ[1] - pZ[0] + pZ[Xext + 1] - pZ[Xext];;
    bx = pX[0] + pX[1] - pX[Xext] - pX[Xext + 1];
    by = pY[0] + pY[1] - pY[Xext] - pY[Xext + 1];
    bz = pZ[0] + pZ[1] - pZ[Xext] - pZ[Xext + 1];
    nx = (ay * bz) - (az * by);
    ny = (az * bx) - (ax * bz);
    nz = (ax * by) - (ay * bx);
    ax = (nx * nx) + (ny * ny) + (nz * nz);
    if (ax == 0.0f) ax = 1.0f;
    ax = 1.0f / sqrtf(ax);
    pNx[x] = nx * ax;
    pNy[x] = ny * ax;
    pNz[x] = nz * ax;
  }
  else
  {
    pNx[x] = 0.0f;
    pNy[x] = 0.0f;
    pNz[x] = 0.0f;
  }
  pNx += Xext;
  pNy += Xext;
  pNz += Xext;
  pValid += Xext;
  pX -= (Xext - 2);
  pY -= (Xext - 2);
  pZ -= (Xext - 2);

// ****** Rows between the topmost and bottommost row ******
  for (y = 1; y < (Yext - 1); y++)
  {

// Leftmost column (6-pixel neighbourhood).
    if (pValid[0] != 0)
    {
      ax = pX[1] - pX[0] + pX[Xext + 1] - pX[Xext] + pX[Xext2 + 1] - pX[Xext2];
      ay = pY[1] - pY[0] + pY[Xext + 1] - pY[Xext] + pY[Xext2 + 1] - pY[Xext2];
      az = pZ[1] - pZ[0] + pZ[Xext + 1] - pZ[Xext] + pZ[Xext2 + 1] - pZ[Xext2];
      bx = pX[0] + pX[1] - pX[Xext2] - pX[Xext2 + 1];
      by = pY[0] + pY[1] - pY[Xext2] - pY[Xext2 + 1];
      bz = pZ[0] + pZ[1] - pZ[Xext2] - pZ[Xext2 + 1];
      nx = (ay * bz) - (az * by);
      ny = (az * bx) - (ax * bz);
      nz = (ax * by) - (ay * bx);
      ax = (nx * nx) + (ny * ny) + (nz * nz);
      if (ax == 0.0f) ax = 1.0f;
      ax = 1.0f / sqrtf(ax);
      pNx[0] = nx * ax;
      pNy[0] = ny * ax;
      pNz[0] = nz * ax;
    }
    else
    {
      pNx[0] = 0.0f;
      pNy[0] = 0.0f;
      pNz[0] = 0.0f;
    }

// Columns between the leftmost and rightmost column (9-pixel neighbourhood).
    for (x = 1; x < (Xext - 1); x++)
    {
      if (pValid[x] != 0)
      {
        ax = pX[2] - pX[0] + pX[Xext + 2] - pX[Xext] + pX[Xext2 + 2] - pX[Xext2];
        ay = pY[2] - pY[0] + pY[Xext + 2] - pY[Xext] + pY[Xext2 + 2] - pY[Xext2];
        az = pZ[2] - pZ[0] + pZ[Xext + 2] - pZ[Xext] + pZ[Xext2 + 2] - pZ[Xext2];
        bx = pX[0] + pX[1] + pX[2] - pX[Xext2] - pX[Xext2 + 1] - pX[Xext2 + 2];
        by = pY[0] + pY[1] + pY[2] - pY[Xext2] - pY[Xext2 + 1] - pY[Xext2 + 2];
        bz = pZ[0] + pZ[1] + pZ[2] - pZ[Xext2] - pZ[Xext2 + 1] - pZ[Xext2 + 2];
        nx = (ay * bz) - (az * by);
        ny = (az * bx) - (ax * bz);
        nz = (ax * by) - (ay * bx);
        ax = (nx * nx) + (ny * ny) + (nz * nz);
        if (ax == 0.0f) ax = 1.0f;
        ax = 1.0f / sqrtf(ax);
        pNx[x] = nx * ax;
        pNy[x] = ny * ax;
        pNz[x] = nz * ax;
      }
      else
      {
        pNx[x] = 0.0f;
        pNy[x] = 0.0f;
        pNz[x] = 0.0f;
      }
      pX++;
      pY++;
      pZ++;
    }

// Rightmost column (6-pixel neighbourhood).
    if (pValid[x] != 0)
    {
      ax = pX[1] - pX[0] + pX[Xext + 1] - pX[Xext] + pX[Xext2 + 1] - pX[Xext2];
      ay = pY[1] - pY[0] + pY[Xext + 1] - pY[Xext] + pY[Xext2 + 1] - pY[Xext2];
      az = pZ[1] - pZ[0] + pZ[Xext + 1] - pZ[Xext] + pZ[Xext2 + 1] - pZ[Xext2];
      bx = pX[0] + pX[1] - pX[Xext2] - pX[Xext2 + 1];
      by = pY[0] + pY[1] - pY[Xext2] - pY[Xext2 + 1];
      bz = pZ[0] + pZ[1] - pZ[Xext2] - pZ[Xext2 + 1];
      nx = (ay * bz) - (az * by);
      ny = (az * bx) - (ax * bz);
      nz = (ax * by) - (ay * bx);
      ax = (nx * nx) + (ny * ny) + (nz * nz);
      if (ax == 0.0f) ax = 1.0f;
      ax = 1.0f / sqrtf(ax);
      pNx[x] = nx * ax;
      pNy[x] = ny * ax;
      pNz[x] = nz * ax;
    }
    else
    {
      pNx[x] = 0.0f;
      pNy[x] = 0.0f;
      pNz[x] = 0.0f;
    }
    pNx += Xext;
    pNy += Xext;
    pNz += Xext;
    pValid += Xext;
    pX += 2;
    pY += 2;
    pZ += 2;
  }

// ****** Bottommost row, leftmost column (4-pixel neighbourhood) ******
  if (pValid[0] != 0)
  {
    ax = pX[1] - pX[0] + pX[Xext + 1] - pX[Xext];
    ay = pY[1] - pY[0] + pY[Xext + 1] - pY[Xext];
    az = pZ[1] - pZ[0] + pZ[Xext + 1] - pZ[Xext];
    bx = pX[0] + pX[1] - pX[Xext] - pX[Xext + 1];
    by = pY[0] + pY[1] - pY[Xext] - pY[Xext + 1];
    bz = pZ[0] + pZ[1] - pZ[Xext] - pZ[Xext + 1];
    nx = (ay * bz) - (az * by);
    ny = (az * bx) - (ax * bz);
    nz = (ax * by) - (ay * bx);
    if (ax == 0.0f) ax = 1.0f;
    ax = 1.0f / sqrtf(ax);
    pNx[0] = nx * ax;
    pNy[0] = ny * ax;
    pNz[0] = nz * ax;
  }
  else
  {
    pNx[0] = 0.0f;
    pNy[0] = 0.0f;
    pNz[0] = 0.0f;
  }

// Columns between the leftmost and rightmost column (6-pixel neighbourhood).
  for (x = 1; x < (Xext - 1); x++)
  {
    if (pValid[x] != 0)
    {
      ax = pX[2] - pX[0] + pX[Xext + 2] - pX[Xext];
      ay = pY[2] - pY[0] + pY[Xext + 2] - pY[Xext];
      az = pZ[2] - pZ[0] + pZ[Xext + 2] - pZ[Xext];
      bx = pX[0] + pX[1] + pX[2] - pX[Xext] - pX[Xext + 1] - pX[Xext + 2];
      by = pY[0] + pY[1] + pY[2] - pY[Xext] - pY[Xext + 1] - pY[Xext + 2];
      bz = pZ[0] + pZ[1] + pZ[2] - pZ[Xext] - pZ[Xext + 1] - pZ[Xext + 2];
      nx = (ay * bz) - (az * by);
      ny = (az * bx) - (ax * bz);
      nz = (ax * by) - (ay * bx);
      ax = (nx * nx) + (ny * ny) + (nz * nz);
      if (ax == 0.0f) ax = 1.0f;
      ax = 1.0f / sqrtf(ax);
      pNx[x] = nx * ax;
      pNy[x] = ny * ax;
      pNz[x] = nz * ax;
    }
    else
    {
      pNx[x] = 0.0f;
      pNy[x] = 0.0f;
      pNz[x] = 0.0f;
    }
    pX++;
    pY++;
    pZ++;
  }

// Rightmost column (4-pixel neighbourhood).
  if (pValid[x] != 0)
  {
    ax = pX[1] - pX[0] + pX[Xext + 1] - pX[Xext];
    ay = pY[1] - pY[0] + pY[Xext + 1] - pY[Xext];
    az = pZ[1] - pZ[0] + pZ[Xext + 1] - pZ[Xext];;
    bx = pX[0] + pX[1] - pX[Xext] - pX[Xext + 1];
    by = pY[0] + pY[1] - pY[Xext] - pY[Xext + 1];
    bz = pZ[0] + pZ[1] - pZ[Xext] - pZ[Xext + 1];
    nx = (ay * bz) - (az * by);
    ny = (az * bx) - (ax * bz);
    nz = (ax * by) - (ay * bx);
    ax = (nx * nx) + (ny * ny) + (nz * nz);
    if (ax == 0.0f) ax = 1.0f;
    ax = 1.0f / sqrtf(ax);
    pNx[x] = nx * ax;
    pNy[x] = ny * ax;
    pNz[x] = nz * ax;
  }
  else
  {
    pNx[x] = 0.0f;
    pNy[x] = 0.0f;
    pNz[x] = 0.0f;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DotProduct1(float *pD, float Thrsh)
{
  unsigned int x, y, Xext, Yext;
  float *pNx, *pNy, *pNz, a, b;
  uint8_t *pMask;

  Xext = ResultWidth;
  Yext = ResultHeight;

  pNx = pResult[3];
  pNy = pResult[4];
  pNz = pResult[5];
  pD = pResult[0];
  pMask = pValid3x3;

  for (y = 0; y < (Yext - 1); y++)
  {
    for (x = 0; x < (Xext - 1); x++)
    {
      if ((pMask[x] * pMask[x + 1]) == 0) a = 1.0f;
      else a = (pNx[x] * pNx[x + 1]) + (pNy[x] * pNy[x + 1]) + (pNz[x] * pNz[x + 1]);
      if ((pMask[x] * pMask[x + Xext]) == 0) b = 1.0f;
      else b = (pNx[x] * pNx[x + Xext]) + (pNy[x] * pNy[x + Xext]) + (pNz[x] * pNz[x + Xext]);
      if (b < a) a = b;
      a = sqrtf(1.0f - (a * a));
      if (a < Thrsh) a = 0.0f;
      pD[x] = a;
    }

    x--;
    if ((pMask[x] * pMask[x + 1]) == 0) a = 1.0f;
    else a = (pNx[x] * pNx[x + 1]) + (pNy[x] * pNy[x + 1]) + (pNz[x] * pNz[x + 1]);
    if ((pMask[x + 1] * pMask[x + Xext + 1]) == 0) b = 1.0f;
    else b = (pNx[x + 1] * pNx[x + Xext + 1]) + (pNy[x + 1] * pNy[x + Xext + 1]) + (pNz[x + 1] * pNz[x + Xext + 1]);
    if (b < a) a = b;
    a = sqrtf(1.0f - (a * a));
    if (a < Thrsh) a = 0.0f;
    pD[x + 1] = a;

    pNx += Xext;
    pNy += Xext;
    pNz += Xext;
    pMask += Xext;
    pD += Xext;
  }

  pNx -= Xext;
  pNy -= Xext;
  pNz -= Xext;
  pMask -= Xext;

  for (x = 0; x < (Xext - 1); x++)
  {
    if ((pMask[x + Xext] * pMask[x + Xext + 1]) == 0) a = 1.0f;
    else a = (pNx[x + Xext] * pNx[x + Xext + 1]) + (pNy[x + Xext] * pNy[x + Xext + 1]) + (pNz[x + Xext] * pNz[x + Xext + 1]);
    if ((pMask[x] * pMask[x + Xext]) == 0) b = 1.0f;
    else b = (pNx[x] * pNx[x + Xext]) + (pNy[x] * pNy[x + Xext]) + (pNz[x] * pNz[x + Xext]);
    if (b < a) a = b;
    a = sqrtf(1.0f - (a * a));
    if (a < Thrsh) a = 0.0f;
    pD[x] = a;
  }

  x--;
  if ((pMask[x + Xext] * pMask[x + Xext + 1]) == 0) a = 1.0f;
  else a = (pNx[x + Xext] * pNx[x + Xext + 1]) + (pNy[x + Xext] * pNy[x + Xext + 1]) + (pNz[x + Xext] * pNz[x + Xext + 1]);
  if ((pMask[x + 1] * pMask[x + Xext + 1]) == 0) b = 1.0f;
  else b = (pNx[x + 1] * pNx[x + Xext + 1]) + (pNy[x + 1] * pNy[x + Xext + 1]) + (pNz[x + 1] * pNz[x + Xext + 1]);
  if (b < a) a = b;
  a = sqrtf(1.0f - (a * a));
  if (a < Thrsh) a = 0.0f;
  pD[x + 1] = a;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DotProductWithVector(float nx, float ny, float nz, float *pDP, float Thrsh)
{
  unsigned int Index, LoopCtr;
  float *pNx, *pNy, *pNz, a;

  LoopCtr = ResultWidth * ResultHeight;
  pNx = pResult[3];
  pNy = pResult[4];
  pNz = pResult[5];

  for (Index = 0; Index < LoopCtr; Index++)
  {
    a = fabsf((pNx[Index] * nx) + (pNy[Index] * ny) + (pNz[Index] * nz));
    if (a > 1.0f) a = 1.0f;
    else if (a < Thrsh) a = 0.0f;
    pDP[Index] = a;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool QNode::LeastSquaresPlaneFit(void)
{
  double a[9];
  double b[4];
  double x, y, z, d;
  double PlaneA, PlaneB, PlaneC;
  unsigned int Xext, Yext, Index, LoopCtr;
  float *pX, *pY, *pZ;
  uint8_t *pLabel;

  Xext = ResultHeight;
  Yext = ResultWidth;
  LoopCtr = Xext * Yext;
  pX = pResult[0];
  pY = pResult[1];
  pZ = pResult[2];
  pLabel = pLabelMap;  

// Mark the plane parameters as invalid and initialize the sums.
  PlaneGamma = PlaneD = PlaneNz = PlaneNy = PlaneNx = 0.0;
  for (Index = 0; Index < 9; Index++) a[Index] = 0.0;

// Loop through the 3D points (no check for all zero because this can't happen with labelled points).
  for (Index = 0; Index < LoopCtr; Index++)
  {
    z = (double)pZ[Index];
    if ((z != 0.0) && (pLabel[Index] == 2))
    {
      x = (double)pX[Index];
      y = (double)pY[Index];

      a[0] += x * x;
      a[1] += x * y;
      a[2] += x;
      a[3] += y * y;
      a[4] += y;
      a[5] += 1.0;
      a[6] += x * z;
      a[7] += y * z;
      a[8] += z;
    }
  }

// Partial reults that are multiply used.
  b[0] = (a[3] * a[5]) - (a[4] * a[4]);
  b[1] = (a[1] * a[4]) - (a[2] * a[3]);
  b[2] = (a[0] * a[4]) - (a[1] * a[2]);
  b[3] = (a[4] * a[6]) - (a[2] * a[7]);

// Computation of the factor A.
  PlaneA = b[3] * b[0];
  d = (a[5] * a[7]) - (a[4] * a[8]);
  PlaneA = PlaneA - (d * b[1]);
  d = (a[1] * a[5]) - (a[2] * a[4]);
  d = d * b[1];
  d = (b[2] * b[0]) - d;
  if (d == 0.0) return false;
  PlaneA = PlaneA / d;

// Computation of the factor B.
  if (b[1] == 0.0) return false;
  PlaneB = b[3] - (PlaneA * b[2]);
  PlaneB = PlaneB / b[1];

// Computation of the factor C.
  if (a[2] == 0.0) return false;
  PlaneC = a[6] - (PlaneA * a[0]) - (PlaneB * a[1]);
  PlaneC = PlaneC / a[2];

  d = sqrt(1.0 + (PlaneA * PlaneA) + (PlaneB * PlaneB));
  PlaneNx = PlaneA / d;
  PlaneNy = PlaneB / d;
  PlaneNz = -1.0 / d;
  PlaneD = -PlaneC / d;
  PlaneGamma = atan2(PlaneNz, PlaneNy);

  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DifferenceFromNominalDisparity(float k, float d)
{
  unsigned int x, y, Xext, Yext;
  float *pSrc, *pDst, nom, a;

  Xext = ResultWidth;
  Yext = ResultHeight;
  pSrc = pResult[0];
  pDst = pResult[1];

  for (y = 0; y < Yext; y++)
  {
    nom = (k * (float)y) + d;

    for (x = 0; x < Xext; x++)
    {
      a = pSrc[x];
      if (a != 0.0f)
      {
        a -= nom;
        if (a == 0.0f) a = 0.001f;
      }
      pDst[x] = a;
    }
    pSrc += Xext;
    pDst += Xext;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback function that receives the camera info.
// We get the coordinates of the camera's optical center from here.
// The focal length and the baseline is obtained from the disparity
// image message.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  P_cx = msg->P[2];
  P_cy = msg->P[6];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback function that receives the disparity data.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
// Compute the reduced resolution disparity image, valid 3x3 regions and smooth the disparity image a bit.
    MakeReducedResolutionDisparityImage((float *)(&(msg->image.data[0])));
    GetValid3x3RegionMask();
    Binomial3x3(pResult[0], pResult[1]);

// 3D domain: compute 3D points from disparity, surface normals from 3D points, edges and horizontal patches.
    DisparityTo3D(pResult[1]);
    SurfaceNormals();
    DotProduct1(pResult[0]);//, sinf(20.0f * (M_PI / 180.0f)));
    DotProductWithVector(-0.024476f, -0.450811f, -0.892284f, pResult[1]);//, cosf(20.0f * (M_PI / 180.0f)));

// Visualisation of processing results.
    FLOAT32_to_RGB888(pResult[2], pBuffer, 100.0f);
    SurfaceNormals_to_RGB888(pBuffer + (4 * ResultWidth));
    FLOAT32_to_RGB888(pResult[0], (pBuffer + (8 * ResultWidth)), 255.0f);
    FLOAT32_to_RGB888(pResult[1], (pBuffer + (12 * ResultWidth)), 255.0f);
    Confidence_to_RGB888(pBuffer + (4 * ImgWidth * ResultHeight));
*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& msg)
{
  float k, d;

// Only do something if the flag 'bRun' is true.
  if (bRun)
  {

// Extract parameters of the disparity image from the received message.
    ImgWidth = msg->image.width;
    ImgHeight = msg->image.height;
    ImgBytesPerRow = msg->image.step;
    FocalLength = msg->f;
    Baseline = msg->T;
    MinDisp = msg->min_disparity;
    MaxDisp = msg->max_disparity;

// Compute the reduced resolution disparity image, valid 3x3 regions and smooth the disparity image a bit.
    MakeReducedResolutionDisparityImage((float *)(&(msg->image.data[0])));
    Gradient(pResult[0], pResult[3]);
    MakeInitialLabelMap();
    MakeVDisparityFromLabelMap();
    FindRelevantHistogramBins();
    RemoveGhosts();

Fallback();

//    MarkFloorCandidates();
    DisparityTo3D(pResult[0]);
    FillObstacleGrid();
    FilterObstacleGrid();
    MaskPlatformFromObstacleGrid(1);
    LaserScanFromObstacleGrid();

// Update the timestamp and publish the two virtual laser scans and associated transforms.
  obstacle_trans.header.stamp = msg->header.stamp;
  obstacle_scan.header.stamp = msg->header.stamp;
  (*p_obstacle_broadcaster).sendTransform(obstacle_trans);
  if (active)
    obstacleScanPub.publish(obstacle_scan);

// Visualisation of processing results.
  if (bShowResults)
  {
    FLOAT32_to_RGB888_AutoScale(pResult[2], pBuffer);
    Confidence_to_RGB888(pBuffer + (4 * ResultWidth));
    for (unsigned int u = 0; u < (ResultWidth * ResultHeight); u++)
    if (pResult[3][u] > 4.0f) pResult[3][u] = 4.0f;
    else if (pResult[3][u] < GradientThreshold) pResult[3][u] = 0.0f;
    FLOAT32_to_RGB888_AutoScale(pResult[3], (pBuffer + (8 * ResultWidth)));
    LabelMap_to_RGB888(pBuffer + (12 * ResultWidth));
    k = 0.0f;
    d = 0.0f;
    VDisp_to_RGB888(pHistAccu, (pBuffer + (4 * ImgWidth * ResultHeight)), k, d);
    ObstacleGridToRGB888((pBuffer + (4 * ImgWidth * ResultHeight) + (4 * ResultWidth)), 0);
    ObstacleGridToRGB888((pBuffer + (4 * ImgWidth * ResultHeight) + (8 * ResultWidth)), 1);

    Q_EMIT imageUpdated();
  }

//double duration = ros::Time::now().toSec();
//duration = ros::Time::now().toSec() - duration;
//duration *= 1000000.0;
//printf("- processing time: %iµs\n", (int)duration);

/*
LocalSmoothness(pResult[1], pResult[2]);
MakeVDisparityWithProperGradientAndSmoothness();
FindRelevantHistogramBins();
LeastSquaresPlaneFit();
*/

/*
    DisparityTo3D(pResult[1]);
    LeastSquaresPlaneFit();
    printf("%f*x + %f*y + %f*z = %f | Gamma = %f\n", PlaneNx, PlaneNy, PlaneNz, PlaneD, 180.0 + (PlaneGamma * (180.0 / M_PI)));
*/

/*
    MakeReducedResolutionDisparityImage((float *)(&(msg->image.data[0])));
    GetValid3x3RegionMask();
    MarkProperGradient2();

    Binomial3x3(pResult[0], pResult[1]);
    LocalSmoothness(pResult[1], pResult[2]);

    MakeVDisparityWithProperGradientAndSmoothness();
    FindRelevantHistogramBins();

    unsigned int u, v;
    float *pAuxF;
    uint8_t *pAuxU1, *pAuxU2;
    v = ResultWidth * ResultHeight;
    pAuxF = pResult[2];
    pAuxU1 = pGradientMask;
    pAuxU2 = pLabelMap;
    for (u = 0; u < v; u++)
      if ((pAuxF[u] >= SMOOTHNESS_THRESHOLD) && (pAuxU1[u] == 1) && (pAuxU2[u] == 1)) pAuxU2[u] = 2;    

    DoVDHoughTransform(k, d);
    if ((k == 0.0f) && (d == 0.0f)) Fallback();
*/


/*
    FindHistMax();
    DoVDHoughTransform(k, d);
    DoVDFitLine(k, d);
    if ((k == 0.0f) && (d == 0.0f)) RemoveFloorCheap(NomK, NomD);
    else
    {
      MarkFloorCandidates(k, d);
    }
    EvaluateHorFloorCandidates();
    EvaluateVerFloorCandidates();

    DifferenceFromNominalDisparity(k, d);

    DifferenceFromNominalDisparity(k, d);
    i = ExtendFloorCandidates();
*/
/*
    FLOAT32_to_RGB888(pResult[0], pBuffer, 2.55f);
    Confidence_to_RGB888(pBuffer + (4 * ResultWidth));
    ValidMask_to_RGB888(pBuffer + (8 * ResultWidth));
    GradientMask_to_RGB888(pBuffer + (12 * ResultWidth));

    LabelMap_to_RGB888(pBuffer + (4 * ImgWidth * ResultHeight));
    VDisp_to_RGB888(pHistAccu, (pBuffer + (4 * ImgWidth * ResultHeight) + (4 * ResultWidth)), k, d);
    FLOAT32_to_RGB888(pResult[2], (pBuffer + (4 * ImgWidth * ResultHeight) + (8 * ResultWidth)), 255.0f);

    VDHoughAccu_to_RGB888(pBuffer + (4 * ImgWidth * ResultHeight) + (12 * ResultWidth));
*/
//    DispDiff_to_RGB888(pBuffer + (4 * ImgWidth * ResultHeight));
//    UDisp_to_RGB888((pBuffer + (4 * ImgWidth * ResultHeight)), (ResultHeight - 1), k2, d2);
//    VDisp_to_RGB888(pHistAccu, (pBuffer + (4 * ImgWidth * ResultHeight) + (4 * ResultWidth)), k, d);
//    VDHoughAccu_to_RGB888(pBuffer + (4 * ImgWidth * ResultHeight) + (8 * ResultWidth));
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor of the class - allocated and configures resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv)
{

  active = true;
  unsigned int AllocSize;

  printf("*** TopKinect constructor:\n");

  bRun = false;
  pResult[0] = 0;
  pHistAccu = 0;
  pConfidence = 0;
  bShowResults = false;

  GradientThreshold = 0.5f;
  GridThreshold = 3;

  MakeLaserScanDXDY();

  if (init_argc >= 2) 
  {
	printf("Loading cfg file TopKinect \n");
	bRun = LoadCfgFile(argv[1]);
  }

  if (bRun)
  {
    printf("TopKinect brun is true \n");
    GetVDHoughLimits();
    GetUDHoughLimits();

// Allocate the buffers for the result and local histogram.
    ResultWidth = ImgWidth / 4;
    ResultHeight = ImgHeight / 4;
    printf(" - result: %ux%u\n", ResultWidth, ResultHeight);

    AllocSize = ResultWidth * ResultHeight;
    AllocSize += (ResultWidth * ResultHeight);
    AllocSize += (ResultWidth * ResultHeight);
    AllocSize += (ResultWidth * ResultHeight);
    AllocSize += (ResultWidth * ResultHeight);
    AllocSize += (ResultWidth * ResultHeight);
    AllocSize += ResultHeight;
    AllocSize += ResultHeight;
    AllocSize += ResultHeight;
    AllocSize += VDHoughHeight;
    AllocSize += VDHoughHeight;
    AllocSize += UDHoughHeight;
    AllocSize += UDHoughHeight;
    AllocSize += (HIST_SIZE * ResultHeight);
    printf(" - allocating a buffer with %u float values\n", AllocSize);
    pResult[0] = new float[AllocSize];
    pResult[1] = pResult[0] + (ResultWidth * ResultHeight);
    pResult[2] = pResult[1] + (ResultWidth * ResultHeight);
    pResult[3] = pResult[2] + (ResultWidth * ResultHeight);
    pResult[4] = pResult[3] + (ResultWidth * ResultHeight);
    pResult[5] = pResult[4] + (ResultWidth * ResultHeight);
    MinFloorDisp = pResult[5] + (ResultWidth * ResultHeight);
    MaxFloorDisp = MinFloorDisp + ResultHeight;
    MaxHistDisp = MaxFloorDisp + ResultHeight;
    VDCosLUT = MaxHistDisp + ResultHeight;
    VDSinLUT = VDCosLUT + VDHoughHeight;
    UDCosLUT = VDSinLUT + VDHoughHeight;
    UDSinLUT = UDCosLUT + UDHoughHeight;
    DispAccu = UDSinLUT + UDHoughHeight;
  
    AllocSize = (HIST_SIZE * ResultHeight);
    AllocSize += ResultHeight;
    AllocSize += ResultHeight;
    AllocSize += (VDHoughWidth * VDHoughHeight);
    AllocSize += (UDHoughWidth * UDHoughHeight);
    AllocSize += (GRID_WIDTH * GRID_HEIGHT);
    AllocSize += (GRID_WIDTH * GRID_HEIGHT);
    printf(" - allocating a buffer with %u uint16_t values\n", AllocSize);
    pHistAccu = new uint16_t[AllocSize];
    MaxHistCnt = pHistAccu + (HIST_SIZE * ResultHeight);
    TotalHistCnt = MaxHistCnt + ResultHeight;
    pVDHoughAccu = TotalHistCnt + ResultHeight;
    pUDHoughAccu = pVDHoughAccu + (VDHoughWidth * VDHoughHeight);
    pObstacleGrid = pUDHoughAccu + (UDHoughWidth * UDHoughHeight);
    pObstacleGrid2 = pObstacleGrid + (GRID_WIDTH * GRID_HEIGHT);

    AllocSize = ResultWidth * ResultHeight;
    AllocSize += (ResultWidth * ResultHeight);
    AllocSize += (ResultWidth * ResultHeight);
    printf(" - allocating a buffer with %u uint8_t values\n", AllocSize);
    pConfidence = new uint8_t[AllocSize];
    pLabelMap = pConfidence + (ResultWidth * ResultHeight);
    pValid3x3 = pLabelMap + (ResultWidth * ResultHeight);

    MakeFloorDispLimits();
    MakeVDLUTs();
    MakeUDLUTs();

// Set up the obstacle avoidance virtual laser scan message.
    obstacle_scan.header.frame_id = "/obstacle_link";
    obstacle_scan.angle_min = (float)(-M_PI / 2.0);
    obstacle_scan.angle_max = (float)(M_PI / 2.0);
    obstacle_scan.angle_increment = (float)(M_PI / 180.0);
    obstacle_scan.time_increment = 0.0f;
    obstacle_scan.scan_time = (1.0f / 30.0f); // FIXME: MAKE SURE THIS CORRESPONDS!!!
    obstacle_scan.range_min = 0.1f;
    obstacle_scan.range_max = 2.5f;
    obstacle_scan.ranges.resize(LASER_SCAN_SIZE);
    obstacle_scan.intensities.resize(0);

// Set up the transform for the obstacle avoidance virtual laser scan.
    obstacle_trans.header.frame_id = "base_link";
    obstacle_trans.child_frame_id = "obstacle_link";
    obstacle_trans.transform.translation.x = -0.05;
    obstacle_trans.transform.translation.y = 0.019;
    obstacle_trans.transform.translation.z = 0.5;
    obstacle_trans.transform.rotation.x = 0.0;
    obstacle_trans.transform.rotation.y = 0.0;
    obstacle_trans.transform.rotation.z = 0.0;
    obstacle_trans.transform.rotation.w = 1.0;
  }
  else printf(">>>> usage: %s Params.txt <<<<\n\n", argv[0]);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor of the class - waits for the ROS node to shut down and
// releases the allocated resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::~QNode()
{
  printf("*** TopKinect destructor:\n");

  printf(" - sending no-go to callback and sleeping one second\n");
  bRun = false;
  usleep(999000);
  printf(" - back from sleep\n");

// Wait for the ROS node to shut down.
  printf(" - waiting for ROS to shut down\n");
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait(999000);
  usleep(1);

// Release the buffers for the result and local histogram.
  if (pConfidence != 0)
  {
    printf(" - deleting uint8_t buffer\n");
    delete [] pConfidence;
    pConfidence = 0;
  }

  if (pHistAccu != 0)
  {
    printf(" - deleting uint16_t buffer\n");
    delete [] pHistAccu;
    pHistAccu = 0;
  }

  if (pResult[0] != 0)
  {
    printf(" - deleting float buffer\n");
    delete [] pResult[0];
    pResult[0] = 0;
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
  if (bRun)
  {

// Init the ROS stuff and create subscribers and a publisher.
    ros::init(init_argc, init_argv, "TopKinect");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::CameraInfo>("/headcam/depth_registered/camera_info", 1, &QNode::cameraInfoCallback, this);
    ros::Subscriber sub2 = n.subscribe<stereo_msgs::DisparityImage>("/headcam/depth_registered/disparity", 1, &QNode::disparityCallback, this);

    std::cout << "active " << active << std::endl;
    ros::Subscriber activeSubs = n.subscribe<std_msgs::String>("/headcam/active",1,&QNode::activeCallback,this);

    std::cout << "active " << active << std::endl;
    if(active)
    {
     	obstacleScanPub = n.advertise<sensor_msgs::LaserScan>("obstacle_scan", 1);
        printf("Publishing obstacle_scan \n");
    }

// Create tf broadcasters for the transform laser to mobile platform.
    tf::TransformBroadcaster obstacle_broadcaster;
    p_obstacle_broadcaster = &obstacle_broadcaster;

// ROS main loop.
    ros::spin();
  }

// Send the shutdown signal.
  printf("Ros shutdown, proceeding to close the gui.\n");
  Q_EMIT rosShutdown();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::FLOAT32_to_RGB888(float *pIn, uint8_t *pOut, float Scale)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  float a;
  uint8_t uc;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      a = pIn[x] * Scale;
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
void QNode::FLOAT32_to_RGB888_AutoScale(float *pIn, uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  float a, MinVal, MaxVal;
  uint8_t uc;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);

  MinVal = 1000000.0f;
  MaxVal = -1000000.0f;
  y = Xext * Yext;
  for (x = 0; x < y; x++)
  {
    a = pIn[x];
    if (a < MinVal) MinVal = a;
    if (a > MaxVal) MaxVal = a;
  }
  MaxVal -= MinVal;
  if (MaxVal == 0.0f) return;
  MaxVal = 255.0f / MaxVal;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      uc = (uint8_t)((pIn[x] - MinVal) * MaxVal);
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
void QNode::UINT16_to_RGB888(uint16_t *pIn, uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  uint16_t a, MinVal, MaxVal;
  float Scale;
  uint8_t uc;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);

  MinVal = 0xffff;
  MaxVal = 0x0000;
  y = Xext * Yext;
  for (x = 0; x < y; x++)
  {
    a = pIn[x];
    if (a < MinVal) MinVal = a;
    if (a > MaxVal) MaxVal = a;
  }
  MaxVal -= MinVal;
  if (MaxVal == 0) return;
  Scale = 255.0f / (float)MaxVal;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      uc = (uint8_t)((float)(pIn[x] - MinVal) * Scale);
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
void QNode::SurfaceNormals_to_RGB888(uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  float *pNx, *pNy, *pNz, a, b, c;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);
  pNx = pResult[3];
  pNy = pResult[4];
  pNz = pResult[5];

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      a = pNx[x];
      b = pNy[x];
      c = pNz[x];
      if ((a != 0.0f) || (b != 0.0f) || (c != 0.0f))
      {
        a = (a + 1.0f) * 127.5f;
        b = (b + 1.0f) * 127.5f;
        c = (c + 1.0f) * 127.5f;
      }
      pOut[0] = (uint8_t)c; // B
      pOut[1] = (uint8_t)b; // G
      pOut[2] = (uint8_t)a; // R
      pOut[3] = 0xff; // A

      pOut += 4;
    }

    pNx += Xext;
    pNy += Xext;
    pNz += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::VDisp_to_RGB888(uint16_t *pIn, uint8_t *pOut, float k, float d)
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
    i = (int)roundf((k * (float)y) + d);
    if (i < 0) i = 0;
    i *= 4;
    pAux8[i + 0] = 0x00; // B
    pAux8[i + 1] = 0x00; // G
    pAux8[i + 2] = 0xff; // R
*/
    i = (int)MinFloorDisp[y];
    if (i < 0) i = 0;
    i *= 4;
    pAux8[i + 0] = 0x00; // B
    pAux8[i + 1] = 0xff; // G
    pAux8[i + 2] = 0x00; // R

    i = (int)MaxFloorDisp[y];
    if (i < 0) i = 0;
    i *= 4;
    pAux8[i + 0] = 0x00; // B
    pAux8[i + 1] = 0xff; // G
    pAux8[i + 2] = 0x00; // R

    if (MaxHistDisp[y] != 0.0f)
    {
      i = (int)roundf(MaxHistDisp[y]);
      i *= 4;
      pAux8[i + 0] = 0x00; // B
      pAux8[i + 1] = 0x00; // G
      pAux8[i + 2] = 0xff; // R
    }
    else
    {
      i = (int)roundf((NomK * (float)y) + NomD);
      i *= 4;
      pAux8[i + 0] = 0x00; // B
      pAux8[i + 1] = 0xff; // G
      pAux8[i + 2] = 0xff; // R
    }

    pIn += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::Confidence_to_RGB888(uint8_t *pOut)
{
  unsigned int x, y, Xext, Yext, OutOffset;
  float a, b;
  uint8_t *pIn;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);
  pIn = pConfidence;

  a = 16.0f / 3.0f;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      b = (float)pIn[x];

// Calculate the blue component.
      if (b <= (2.0f * a)) pOut[0] = 0;
      else pOut[0] = (uint8_t)roundf((b - (2.0f * a)) * (255.0f / a));

// Calculate the green component.
      if (b <= a) pOut[1] = 0;
      else if (b >= (2.0f * a)) pOut[1] = 255;
      else pOut[1] = (uint8_t)roundf((b - a) * (255.0f / a));

// Calculate the red component.
      if (b >= a) pOut[2] = 255;
      else pOut[2] = (uint8_t)roundf(b * (255.0f / a));

// Alpha channel is always 255.
      pOut[3] = 0xff;
      pOut += 4;
    }

    pIn += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::ValidMask_to_RGB888(uint8_t *pOut)
{
  unsigned int x, y, Xext, Yext, OutOffset;
  uint8_t *pIn, uc;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);
  pIn = pValid3x3;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      uc = pIn[x];
      if (uc > 0) uc = 255;
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
void QNode::VDHoughAccu_to_RGB888(uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  uint8_t uc;
  uint16_t *pIn, Max;
  float a;

  Xext = VDHoughWidth;
  Yext = VDHoughHeight;
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
void QNode::LabelMap_to_RGB888(uint8_t *pOut)
{
  unsigned int x, y, Xext, Yext, OutOffset;
  uint8_t *pIn, uc;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);
  pIn = pLabelMap;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      uc = 63 * pIn[x];
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
void QNode::UDisp_to_RGB888(uint8_t *pOut, unsigned int RowNumber, float k, float d)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  int Index;
  uint8_t *pAux8;
  float a, b, *pDisp;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);

  pAux8 = pOut;
  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      pAux8[0] = 0x00; // B
      pAux8[1] = 0x00; // G
      pAux8[2] = 0x00; // R
      pAux8[3] = 0xff; // A
      pAux8 += 4;
    }
    pAux8 += OutOffset;
  }

  pAux8 = pOut;
  pDisp = pResult[0] + (ResultWidth * RowNumber);
  b = (float)(ResultWidth / 2);

  for (x = 0; x < ResultWidth; x++)
  {
    Index = (int)roundf(pDisp[x]);
    Index = (4 * x) + (Index * DispStep);
    pAux8[Index] = 0xff; // B
    pAux8[Index + 1] = 0xff; // G
    pAux8[Index + 2] = 0xff; // R
    pAux8[Index + 3] = 0xff; // A

    a = (float)x - b;
    Index = (int)roundf((k * a) + d);
    if ((Index >= 0) && (Index < (int)ResultHeight))
    {
      Index = (4 * x) + (Index * DispStep);
      pAux8[Index] = 0x00; // B
      pAux8[Index + 1] = 0x00; // G
      pAux8[Index + 2] = 0xff; // R
      pAux8[Index + 3] = 0xff; // A
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::DispDiff_to_RGB888(uint8_t *pOut)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  float a, *pIn, Min, Max;
  uint8_t uc, *pLabel;

  Xext = ResultWidth;
  Yext = ResultHeight;
  OutOffset = DispStep - (4 * Xext);
  pLabel = pLabelMap;

  pIn = pResult[1];
  Min = 1000000.0f;
  Max = -1000000.0f;
  y = Xext * Yext;
  for (x = 0; x < y; x++)
  {
    if ((pLabel[x] != 0) && (pLabel[x] != 3))
    {
      a = pIn[x];
      if (a < Min) Min = a;
      if (Max < a) Max = a;
    }
  }

  if ((Min == 1000000.0f) && (Max == -1000000.0f)) return;
  else Max -= Min;
  if (Max == 0.0f) return;
  else Max = 255.0f / Max;

  for (y = 0; y < Yext; y++)
  {
    for (x = 0; x < Xext; x++)
    {
      uc = 0;
      if ((pLabel[x] != 0) && (pLabel[x] != 3)) uc = (uint8_t)((pIn[x] - Min) * Max);
      pOut[0] = uc; // B
      pOut[1] = uc; // G
      pOut[2] = uc; // R
      pOut[3] = 0xff; // A

      pOut += 4;
    }

    pIn += Xext;
    pLabel += Xext;
    pOut += OutOffset;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::ObstacleGridToRGB888(uint8_t *pOut, unsigned int Selector)
{
  uint32_t x, y, Xext, Yext, OutOffset;
  uint8_t uc;
  uint16_t *pIn, Max;

  Xext = GRID_WIDTH;
  Yext = GRID_HEIGHT;
  OutOffset = DispStep - (4 * Xext);
  if (Selector == 0) pIn = pObstacleGrid;
  else pIn = pObstacleGrid2;

  y = Xext * Yext;
  Max = 0;
  for (x = 0; x < y; x++)
    if (Max < pIn[x]) Max = pIn[x];

  if (Max == 0)
  {
    for (y = 0; y < Yext; y++)
    {
      for (x = 0; x < Xext; x++)
      {
        pOut[0] = 0; // B
        pOut[1] = 0; // G
        pOut[2] = 0x7f; // R
        pOut[3] = 0xff; // A
        pOut += 4;
      }
      pOut += OutOffset;
    }
  }
  else
  {
    pIn += ((Xext * Yext) - 1);

    for (y = 0; y < Yext; y++)
    {
      for (x = 0; x < Xext; x++)
      {
        if (*pIn == 0)
        {
          pOut[0] = 0; // B
          pOut[1] = 0; // G
          pOut[2] = 0x7f; // R
          pOut[3] = 0xff; // A
        }
        else if (*pIn == 1)
        {
          pOut[0] = 0x7f; // B
          pOut[1] = 0; // G
          pOut[2] = 0; // R
          pOut[3] = 0xff; // A
        }
        else
        {
          uc = (uint8_t)(255.0f * ((float)(*pIn) / (float)Max));
          pOut[0] = uc; // B
          pOut[1] = uc; // G
          pOut[2] = uc; // R
          pOut[3] = 0xff; // A
        }

        pIn--;
        pOut += 4;
      }

      pOut += OutOffset;
    }
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::activeCallback(const std_msgs::String::ConstPtr& msg)
{

	std::cout << "top kinect active callback " << msg->data << std::endl;
	if(msg->data.compare("inactive") == 0)
		active = false;

	if(msg->data.compare("active") == 0)
		active = true;

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

