
#include "config.h"
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/CameraPinhole.h"
#include "../src/libmv_mapping.h"
#include "../src/ProjectiveEstimator.h"

#include <opencv2/calib3d/calib3d.hpp>


//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////


#include "test_data_sets.h"

NEW_TUTO(Proj_Rec, "Projective reconstruction",
  "Using points and intra parameters, try to compute motion and object"){

  //universal method to get the current image:
  vector<Mat> images;

  MotionProcessor mp;
  mp.setInputSource(FROM_SRC_ROOT("Medias/temple/"),IS_DIRECTORY);

  vector<PointOfView> myCameras =
    loadCamerasFromFile(FROM_SRC_ROOT("Medias/temple/temple_par.txt"),
    LOAD_INTRA);
  vector<PointOfView> myCamerasReal =
    loadCamerasFromFile(FROM_SRC_ROOT("Medias/temple/temple_par.txt"),
    LOAD_FULL);
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

  string pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_SIFT/motion_tracks.yml");
  std::ifstream inPoints(pathFileTracks.c_str());
  if( !inPoints.is_open() )
  {
    pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_SIFT/motion_tracks1.yml");
    inPoints.open(pathFileTracks.c_str());
    if( !inPoints.is_open() )
    {
      cout<<"please compute points matches using testMotionEstimator.cpp first!"<<endl;
      return;
    }
    else
    {
      FileStorage fsRead(pathFileTracks, FileStorage::READ);
      FileNode myPtt = fsRead.getFirstTopLevelNode();
      SequenceAnalyzer motion_tmp( images, myPtt );
      fsRead.release();
      motion_tmp.keepOnlyCorrectMatches();
      pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_SIFT/motion_tracks.yml");
      //now save the tracks:
      FileStorage fsOutMotion(pathFileTracks, FileStorage::WRITE);
      //Can't find a way to enable the following notation:
      //fs << *ptt1;
      SequenceAnalyzer::write(fsOutMotion,motion_tmp);
      fsOutMotion.release();
    }
  }
  inPoints.close();

  FileStorage fsRead(pathFileTracks, FileStorage::READ);
  FileNode myPtt = fsRead.getFirstTopLevelNode();
  SequenceAnalyzer motion_estim_loaded( images, myPtt );
  fsRead.release();

  vector<TrackOfPoints> &tracks=motion_estim_loaded.getTracks();
  cout<<"numbers of correct tracks loaded:"<<tracks.size()<<endl;

  //now for fun show the sequence on images:
  //motion_estim_loaded.showTracks(0);

  //myCameras contains only intra value. I will use motion_estim_loaded to
  //compute position of cameras:
  ProjectiveEstimator pe(motion_estim_loaded, myCameras);

  pe.computeReconstruction(myCamerasReal);
}