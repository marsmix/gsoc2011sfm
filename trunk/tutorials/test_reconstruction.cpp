
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/StructureEstimator.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include "../src/libmv_mapping.h"
#include "../src/config.h"
#include <opencv2/calib3d/calib3d.hpp>


//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Only to see how we can create a 3D structure estimation using calibrated cameras
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

NEW_TUTO(Triangulation_tuto, "Learn how you can triangulate 2D points",
  "Using fully parameterized cameras, we find 2D points in the sequence and then triangulate them. We finally draw them on the sequence.")
{
  //universal method to get the current image:
  vector<Mat> images;

  string pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_SIFT/motion_tracks.yml");
  std::ifstream inPoints(pathFileTracks.c_str());
  if( !inPoints.is_open() )
  {
    cout<<"please compute points matches using testMotionEstimator.cpp first!"<<endl;
    return;
  }
  inPoints.close();
  
  cout<<"First load the cameras from Medias/temple/temple_par.txt"<<endl;
  vector<PointOfView> myCameras=loadCamerasFromFile(FROM_SRC_ROOT("Medias/temple/temple_par.txt"));
  MotionProcessor mp;
  mp.setInputSource(FROM_SRC_ROOT("Medias/temple/"),IS_DIRECTORY);

  cout<<"Then load all images from Medias/temple/"<<endl;
  vector<PointOfView>::iterator itPoV=myCameras.begin();
  int index_image=-1;
  while ( itPoV!=myCameras.end() )
  {
    Mat imgTmp=mp.getFrame();//get the current image
    if(imgTmp.empty())
      break;//end of sequence: quit!
    index_image++;
    images.push_back(imgTmp);
  }

  cout<<"Finally create a new PointsToTrack using Medias/tracks_points_SIFT/motion_tracks.yml"<<endl;
  FileStorage fsRead(pathFileTracks, FileStorage::READ);
  FileNode myPtt = fsRead.getFirstTopLevelNode();
  SequenceAnalyzer motion_estim_loaded( images, myPtt );
  fsRead.release();

  cout<<"numbers of correct tracks loaded:"<<
    motion_estim_loaded.getTracks().size()<<endl;

  int maxImg=motion_estim_loaded.getNumViews();

  cout<<"triangulation of points."<<endl;
  StructureEstimator structure (motion_estim_loaded, myCameras);
  vector<TrackPoints> tracks;
  structure.computeStructure(tracks);
  cout<<tracks.size()<<" points found."<<endl;

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
    //motion_estim_loaded.filterPoints(triangulated,index_image,points3D,points2DOrigine);
    vector<Vec2d> pixelProjected=itPoV->project3DPointsIntoImage(tracks);
    //convert Vec2d into KeyPoint:
    vector<KeyPoint> points2D;
    for(unsigned int j=0;j<pixelProjected.size();j++)
      points2D.push_back( KeyPoint( (float)pixelProjected[j][0],
      (float)pixelProjected[j][1], 10.0 ) );

    Mat imgTmp2;
    drawKeypoints(imgTmp,points2D,imgTmp2,Scalar(255,255,255));
    imshow("Points projected...",imgTmp2);
    cv::waitKey(0);
    itPoV++;
  }
  //now for fun show the sequence on images:
  motion_estim_loaded.showTracks(0);
}
