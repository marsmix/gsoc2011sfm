//Set to 1 if you want to test the cameras
//But be aware to set other tests to 0...
#if 1

#include "CameraPinholeDistor.h"
#include <iostream>

using namespace std;
using namespace OpencvSfM;
using namespace cv;

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Problems with DISTORTION: unable to find a fast and accurate way to
//transform pixels in camera normalized coordinates into image pixels.
//The only way I found was to use LM iteration to find the original point...
//////////////////////////////////////////////////////////////////////////
void main()
{
  Mat intra=Mat::eye(3, 3, CV_64F);
  double* data = (double*)intra.data;

  data[0]=1500.0;
  data[4]=1535.0;
  data[2]=320.0;
  data[5]=240.0;

  Vec6d radialDist(0.2,0.4,0.3,0.1,0.1,0.01);
  Vec2d tangDist(0.01,0.1);

  CameraPinholeDistor camera(intra,radialDist,6,tangDist);
  //CameraPinhole camera(intra);

  vector<Vec2d> points;//image pixel coords.

  points.push_back(Vec2d(10,10));
  points.push_back(Vec2d(100,10));
  points.push_back(Vec2d(20,100));
  points.push_back(Vec2d(250,500));
  points.push_back(Vec2d(130,10));
  points.push_back(Vec2d(450,200));

  vector<Vec2d> pointsImgCoord;//image coords.
  pointsImgCoord=camera.pixelToNormImageCoordinates(points);
  for(int i=0;i<6;i++)
  {
    cout<<pointsImgCoord[i][0]<<" "<<pointsImgCoord[i][1]<<endl;
  }

  vector<Vec2d> pointsImgPixel;//back to pixel coord.
  pointsImgPixel=camera.normImageToPixelCoordinates(pointsImgCoord);
  for(int i=0;i<6;i++)
  {
    cout<<pointsImgPixel[i][0]<<" "<<pointsImgPixel[i][1]<<endl;
  }

  //////////////////////////////////////////////////////////////////////////
  //Compute projection matrix
  //////////////////////////////////////////////////////////////////////////
  double m[3][3] = {{5, 2, 1}, {2, 3, 4}, {2, 5, 1}};
  cv::Mat rotation = cv::Mat(3, 3, CV_64F, m);
  Vec3d translation(5,10,20);
  Mat projectionMat=camera.computeProjectionMatrix(rotation,translation);

}
#endif