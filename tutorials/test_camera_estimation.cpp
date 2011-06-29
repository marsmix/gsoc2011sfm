//Set to 1 if you want to test the points detection and matching
//But be aware to set other tests to 0...
#if 1

#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include "../src/libmv_mapping.h"
#include "../src/ProjectiveEstimator.h"
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
enum{LOAD_INTRA=1,LOAD_POSITION=2,LOAD_FULL=3};
vector<PointOfView> loadCamerasFromFile(string fileName, int flag_model)
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
        if( (flag_model & LOAD_FULL) == LOAD_FULL)
        {
          outVect.push_back(
            PointOfView(new CameraPinhole(intra_params),rotation,translation));
        }
        else
        {
          if( (flag_model & LOAD_FULL) == LOAD_POSITION)
          {
            outVect.push_back(
              PointOfView(new CameraPinhole(),rotation,translation));
          }
          else
          {
            if( (flag_model & LOAD_FULL) == LOAD_INTRA)
            {
              outVect.push_back(
                PointOfView(new CameraPinhole(intra_params)));
            }
          }
        }
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
  MotionProcessor mp;
  mp.setInputSource("../Medias/temple/",IS_DIRECTORY);

  vector<PointOfView> myCameras=loadCamerasFromFile("../Medias/temple/temple_par.txt",
    LOAD_INTRA);
  vector<PointOfView>::iterator itPoV=myCameras.begin();
  int index_image=-1;
  while (itPoV!=myCameras.end() )
  {
    Mat imgTmp=mp.getFrame();//get the current image
    if(imgTmp.empty())
      break;//end of sequence: quit!
    index_image++;
    images.push_back(imgTmp);
  }

  //and create a new PointsToTrack using this file:
  vector<Ptr<PointsToTrack>> points_empty;
  SequenceAnalyzer motion_estim_loaded( images, points_empty, matches_algo );

  FileStorage fsRead("motion_tracks.yml", FileStorage::READ);
  FileNode myPtt = fsRead.getFirstTopLevelNode();
  SequenceAnalyzer::read(myPtt, motion_estim_loaded);
  fsRead.release();

  vector<TrackPoints> &tracks=motion_estim_loaded.getTracks();
  cout<<"numbers of correct tracks loaded:"<<tracks.size()<<endl;

  //now for fun show the sequence on images:
  //motion_estim_loaded.showTracks(0);

  //myCameras contains only intra value. I will use motion_estim_loaded to
  //compute position of cameras:
  ProjectiveEstimator pe(motion_estim_loaded, myCameras);

  pe.comptueReconstruction();
}


#endif