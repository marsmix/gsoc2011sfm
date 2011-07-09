
#include "../src/CameraPinhole.h"
#include "../src/PointOfView.h"
#include <iostream>

using namespace std;
using namespace OpencvSfM;
using namespace cv;

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

NEW_TUTO(Setup_Camera, "Learn how setup cameras",
  "Using intra parameters and positions of cameras, we project 3D poins into cameras' coordinates.")
{
  //create an arbitrary camera (from data/temple_par.txt) :
  Mat intra=Mat::eye(3, 3, CV_64F);
  double* data = (double*)intra.data;
  data[0]=1520.4;
  data[2]=302.32;
  data[4]=1525.9;
  data[5]=246.87;
  Ptr<Camera> camera =  Ptr<Camera>(new CameraPinhole(intra));
  //Now create a real camera (ie with a position) using the same file:
  Mat rotation(3,3,CV_64F);
  data = (double*)rotation.data;
  data[0] = 0.015513720929994632;  data[1] = 0.99884343581246959;   data[2] = -0.045509506668906109;
  data[3] = 0.99922238739871228;   data[4] = -0.017137499028595668; data[5] = -0.035509528978323907;
  data[6] = -0.036248379055121745; data[7] = -0.044923232980116717; data[8] = -0.99833258894743582;
  Vec3d translation;
  translation[0]=-0.059985479001418429;translation[1]=0.0040078802950409987;translation[2]=0.57088647431543438;

  PointOfView firstPoV( camera, rotation, translation );

  //As the (tight) bounding box for the temple model is (-0.054568 0.001728 -0.042945) - (0.047855 0.161892 0.032236)
  //I will create some 3D points and see if they are correcly reprojected:
  vector<Vec3d> points3D;
  points3D.push_back(Vec3d( -0.054568, 0.001728, -0.042945 ));
  points3D.push_back(Vec3d( -0.044568, 0.011728, -0.032945 ));
  points3D.push_back(Vec3d( -0.034568, 0.041728, -0.012945 ));
  points3D.push_back(Vec3d( -0.024568, 0.081728, -0.002945 ));
  points3D.push_back(Vec3d( -0.014568, 0.121728,  0.012945 ));
  points3D.push_back(Vec3d( -0.004568, 0.161728,  0.032945 ));

  vector<Vec2d> pixelProjection=firstPoV.project3DPointsIntoImage(points3D);

  for(unsigned int i=0;i<pixelProjection.size();i++){
    cout<<"Point "<<i<<" projected in ("<<pixelProjection[i][0]<<", "<<pixelProjection[i][1]<<")"<<endl;
    Vec4d point3DHomog(points3D[i][0],points3D[i][1],points3D[i][2],1);
  }
}