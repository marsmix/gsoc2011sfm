//Set to 1 if you want to test the points detection and matching
//But be aware to set other tests to 0...
#if 0

#include "PointsToTrackWithImage.h"
#include "MotionProcessor.h"
#include "PointOfView.h"
#include "CameraPinhole.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>

#include <iostream>

using namespace std;
using namespace cv;
using namespace OpencvSfM;

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

vector<PointOfView> loadCamerasFromFile(string fileName)
{
  vector<PointOfView> outVect;
  ifstream pointsDef(fileName);
  bool isOK=pointsDef.is_open();
  //first get the numbers of cameras:
  int nbCameras;
  if(pointsDef>>nbCameras)
  {
    string name_of_picture;
    Mat intra_params,rotation;
    Vec3d translation;
    intra_params.create(3, 3, CV_64F);
    rotation.create(3, 3, CV_64F);
    double* data_intra_param=(double*)intra_params.data;
    double* data_rotation=(double*)rotation.data;
    for (int i=0;i<nbCameras;i++)
    {
      //first the name of image:
      if(pointsDef>>name_of_picture)
      {
        //the 9 values of K:
        for(int j=0;j<9;j++)
          pointsDef>>data_intra_param[j];
        //the 9 values of rotation:
        for(int j=0;j<9;j++)
          pointsDef>>data_rotation[j];
        //the 3 values of translation:
        for(int j=0;j<3;j++)
          pointsDef>>translation[j];
        //now create a point of view:
        outVect.push_back(PointOfView(new CameraPinhole(intra_params),rotation,translation));
      }
    }
  }
  return outVect;
}

void main(){
  vector<PointOfView> myCameras=loadCamerasFromFile("../Medias/temple/temple_par.txt");

  //As the (tight) bounding box for the temple model is (-0.054568 0.001728 -0.042945) - (0.047855 0.161892 0.032236)
  //I will create some 3D points and see if they are correcly reprojected:

  vector<Vec3d> points3D;
  points3D.push_back(Vec3d( -0.054568, 0.001728, -0.042945 ));
  points3D.push_back(Vec3d( -0.054568, 0.001728, 0.032236 ));
  points3D.push_back(Vec3d( 0.047855, 0.001728, -0.042945 ));
  points3D.push_back(Vec3d( -0.054568, 0.161892, -0.042945 ));
  points3D.push_back(Vec3d( -0.054568, 0.161892, 0.032236 ));
  points3D.push_back(Vec3d( 0.047855, 0.001728, 0.032236 ));
  points3D.push_back(Vec3d( 0.047855, 0.161892, -0.042945 ));
  points3D.push_back(Vec3d( 0.047855, 0.161892, 0.032236 ));

  //now for each point of view, we draw the picture and these points projected:
  MotionProcessor mp;
  //Here we will a folder with a lot of images, but we can do the same thing with any other type of input
  mp.setInputSource("../Medias/temple/",IS_DIRECTORY);

  vector<PointOfView>::iterator itPoV=myCameras.begin();
  while (itPoV!=myCameras.end())
  {
    Mat imgTmp=mp.getFrame();//get the current image
    if(imgTmp.empty())
      break;//end of sequence: quit!

    vector<Vec2d> pixelProjected=itPoV->project3DPointsIntoImage(points3D);
    //convert Vec2d into KeyPoint:
    vector<KeyPoint> points2D;
    for(unsigned int j=0;j<pixelProjected.size();j++)
      points2D.push_back(KeyPoint((float)pixelProjected[j][0],(float)pixelProjected[j][1],10.0));

    drawKeypoints(imgTmp,points2D,imgTmp,Scalar(255,255,255));
    imshow("Points projected...",imgTmp);
    cv::waitKey(0);
    cv::waitKey(40);
    itPoV++;
  }
}

#endif