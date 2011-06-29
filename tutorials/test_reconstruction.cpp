//Set to 1 if you want to test the points detection and matching
//But be aware to set other tests to 0...
#if 0

#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include "../src/libmv_mapping.h"
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace std;
using namespace cv;
using namespace OpencvSfM;

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Only to see how we can create a 3D structure estimation using calibrated cameras
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
  Ptr<DescriptorMatcher> matcher;
  matcher=Ptr<DescriptorMatcher>(new FlannBasedMatcher());
  Ptr<PointsMatcher> matches_algo ( new PointsMatcher(matcher) );

  //universal method to get the current image:
  vector<Mat> images;

  if( !boost::filesystem::exists( boost::filesystem::status("motion_tracks.yml") ) )
  {
    cout<<"please compute points matches using testMotionEstimator.cpp first!"<<endl;
    return;
  }

  //and create a new PointsToTrack using this file:
  vector<Ptr<PointsToTrack>> points_empty;
  MotionEstimator motion_estim_loaded( points_empty, matches_algo );

  FileStorage fsRead("motion_tracks.yml", FileStorage::READ);
  FileNode myPtt = fsRead.getFirstTopLevelNode();
  MotionEstimator::read(myPtt, motion_estim_loaded);
  fsRead.release();

  vector<TrackPoints> &tracks=motion_estim_loaded.getTracks();
  cout<<"numbers of correct tracks loaded:"<<tracks.size()<<endl;

  vector<PointOfView> myCameras=loadCamerasFromFile("../Medias/temple/temple_par.txt");
  vector<Vec3d> triangulated;
  MotionProcessor mp;
  mp.setInputSource("../Medias/temple/",IS_DIRECTORY);
  
  vector<PointOfView>::iterator itPoV=myCameras.begin();
  int index_image=-1, maxImg=motion_estim_loaded.getNumViews();
  while (itPoV!=myCameras.end() && index_image<maxImg )
  {
    Mat imgTmp=mp.getFrame();//get the current image
    if(imgTmp.empty())
      break;//end of sequence: quit!
    index_image++;
    images.push_back(imgTmp);
  }

  cout<<"triangulation of points."<<endl;
  motion_estim_loaded.triangulateNView(myCameras/*,images*/,triangulated);
  cout<<triangulated.size()<<" points found."<<endl;

  //now for each point of view, we draw the picture and these points projected:
  itPoV=myCameras.begin();
  index_image=0;
  while (itPoV!=myCameras.end() && index_image<maxImg )
  {
    Mat imgTmp=images[index_image];//get the current image
    if(imgTmp.empty())
      break;//end of sequence: quit!
    index_image++;

    //create the vector of 3D points viewed by this camera:
    vector<Vec3d> points3D;
    vector<KeyPoint> points2DOrigine;
    //motion_estim_loaded.filterPoints(triangulated,index_image,points3D,points2DOrigine);
    vector<Vec2d> pixelProjected=itPoV->project3DPointsIntoImage(triangulated);
    //convert Vec2d into KeyPoint:
    vector<KeyPoint> points2D;
    for(unsigned int j=0;j<pixelProjected.size();j++)
      points2D.push_back( KeyPoint( (float)pixelProjected[j][0],
      (float)pixelProjected[j][1], 10.0 ) );

    Mat imgTmp1,imgTmp2;
    drawKeypoints(imgTmp,points2DOrigine,imgTmp1,Scalar(255,255,255));
    drawKeypoints(imgTmp,points2D,imgTmp2,Scalar(255,255,255));
    imshow("Points origine...",imgTmp1);
    imshow("Points projected...",imgTmp2);
    cv::waitKey(0);
    itPoV++;
  }
  //now for fun show the sequence on images:
  motion_estim_loaded.showTracks(images,0);
}


#endif