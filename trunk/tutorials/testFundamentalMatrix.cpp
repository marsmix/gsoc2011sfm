
#include "config.h"
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>


//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

NEW_TUTO(Camera_tuto, "Learn how you can mix 3D objects and fully parameterized cameras",
  "Using fully parameterized cameras, we draw a bounding box around the object.")
{
  vector<PointOfView> myCameras=loadCamerasFromFile(FROM_SRC_ROOT("Medias/temple/temple_par.txt"));

  cout<<"As the (tight) bounding box for the temple model is "<<
    "(-0.054568 0.001728 -0.042945) - (0.047855 0.161892 0.032236)"<<
    ", I create 3D points on corners of this boundng box"<<
    "and see if they are correcly reprojected:"<<endl;

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
  //Here we will open a folder with a lot of images
  //but we can do the same thing with any other type of input
  mp.setInputSource(FROM_SRC_ROOT("Medias/temple/"),IS_DIRECTORY);

  cout<<"load images from Medias/temple/ and draw theses points"<<endl;
  namedWindow("Points projected...", CV_WINDOW_AUTOSIZE);
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
    itPoV++;
  }
  cvDestroyWindow("Points projected...");
}