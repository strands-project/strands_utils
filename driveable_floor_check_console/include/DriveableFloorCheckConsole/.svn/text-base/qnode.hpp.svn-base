//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 24.5.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef TopKinect_QNODE_HPP_
#define TopKinect_QNODE_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <stdint.h>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include <QThread>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define HIST_SIZE 		100
#define SMOOTHNESS_THRESHOLD	0.95f
#define RELEVANT_HIT_MIN	8
#define GRID_WIDTH		101
#define GRID_HEIGHT		80
#define GRID_RESOLUTION		0.02f
#define COS_PLANE_GAMMA		0.45880464f
#define SIN_PLANE_GAMMA		0.88853717f
#define LASER_SCAN_SIZE		181
#define LASER_SCAN_OUT_OF_RANGE	1.99f
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinect {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class QNode : public QThread
{
  Q_OBJECT

  private:
    float LaserScanDX[LASER_SCAN_SIZE];
    float LaserScanDY[LASER_SCAN_SIZE];
    int LaserScanLength[LASER_SCAN_SIZE];
    double PlaneNx, PlaneNy, PlaneNz, PlaneD, PlaneGamma;
    float DeltaXrc, DeltaYrc;

    int init_argc;
    char **init_argv;
    bool bRun;

    unsigned int ResultWidth, ResultHeight;
    float *pResult[6];
    float *MinFloorDisp, *MaxFloorDisp;
    float *MaxHistDisp;
    float *DispAccu;
    float MinK, MinD, NomK, NomD, MaxK, MaxD;
    float GradientThreshold;
    uint16_t GridThreshold;

    float VDMinAng, VDMinDist;
    float *VDCosLUT, *VDSinLUT;
    int VDHoughWidth, VDHoughHeight;

    float UDMinAng;
    float *UDCosLUT, *UDSinLUT;
    int UDHoughWidth, UDHoughHeight;

    uint16_t *pHistAccu;
    uint16_t *MaxHistCnt;
    uint16_t *TotalHistCnt;
    uint16_t *pVDHoughAccu;
    uint16_t *pUDHoughAccu;
    uint16_t *pObstacleGrid, *pObstacleGrid2;

    uint8_t *pConfidence;
    uint8_t *pLabelMap;
    uint8_t *pValid3x3;

    sensor_msgs::LaserScan obstacle_scan;
    ros::Publisher obstacleScanPub;
    geometry_msgs::TransformStamped obstacle_trans;
    tf::TransformBroadcaster *p_obstacle_broadcaster;

    bool active;
    //ros::Subscriber activeSubs;
    void activeCallback(const std_msgs::String::ConstPtr& msg);

    void MakeLaserScanDXDY(void);
    bool LoadCfgFile(const char *FileName);
    void GetVDHoughLimits(void);
    void GetUDHoughLimits(void);
    void MakeVDLUTs(void);
    void MakeUDLUTs(void);
    void MakeFloorDispLimits(void);

    void MakeReducedResolutionDisparityImage(float *pDisp);
    void GetValid3x3RegionMask(void);
    void MakeVDisparity(void);
    void MakeVDisparityFromLabelMap(void);
    void Binomial3x3(float *pI, float *pG);
    void Gradient(float *pI, float *pG);
    void LocalSmoothness(float *pI, float *pS);
    void Fallback(void);
    void FillObstacleGrid(void);
    void FilterObstacleGrid(void);
    void MaskPlatformFromObstacleGrid(unsigned int Selector = 0);
    void LaserScanFromObstacleGrid(void);

    void FindRelevantHistogramBins(void);
    void MakeInitialLabelMap(void);

    void RemoveFloorCheap(float k, float d);
    void MarkFloorCandidates(void);
    void RemoveGhosts(void);

    void DoVDHoughTransform(float &k, float &d, int StartRow = 0);
    void DoVDFitLine(float &k, float &d);
    void DoUDHoughTransform(float &k, float &d, int &PointCount, int &MaxCount, unsigned int RowNumber);
    void DoUDFloorLabel(float k, float d, unsigned int RowNumber);
    void DoUDFitLine(float &k, float &d, unsigned int RowNumber);
    void DisparityTo3D(float *pDisp);
    void SurfaceNormals(void);
    void DotProduct1(float *pD, float Thrsh = 0.0f);
    void DotProductWithVector(float nx, float ny, float nz, float *pDP, float Thrsh = 0.0f);
    bool LeastSquaresPlaneFit(void);

    void DifferenceFromNominalDisparity(float k, float d);

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& msg);

  public:
    uint8_t *pBuffer;
    unsigned int ImgWidth, ImgHeight;
    float FocalLength, Baseline, P_cx, P_cy, MinDisp, MaxDisp;
    unsigned int ImgBytesPerRow;
    unsigned int DispStep;
    bool bShowResults;

    QNode(int argc, char **argv);
    virtual ~QNode();
    bool init();
    void run();

    void FLOAT32_to_RGB888(float *pIn, uint8_t *pOut, float Scale = 1.0f);
    void FLOAT32_to_RGB888_AutoScale(float *pIn, uint8_t *pOut);
    void UINT16_to_RGB888(uint16_t *pIn, uint8_t *pOut);
    void SurfaceNormals_to_RGB888(uint8_t *pOut);
    void VDisp_to_RGB888(uint16_t *pIn, uint8_t *pOut, float k = 0.0f, float d = 0.0f);
    void Confidence_to_RGB888(uint8_t *pOut);
    void GradientMask_to_RGB888(uint8_t *pOut);
    void ValidMask_to_RGB888(uint8_t *pOut);
    void VDHoughAccu_to_RGB888(uint8_t *pOut);
    void LabelMap_to_RGB888(uint8_t *pOut);
    void UDisp_to_RGB888(uint8_t *pOut, unsigned int RowNumber, float k = 0.0f, float d = 0.0f);
    void DispDiff_to_RGB888(uint8_t *pOut);
    void ObstacleGridToRGB888(uint8_t *pOut, unsigned int Selector = 0);

  Q_SIGNALS:
    void imageUpdated();
    void rosShutdown();
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

