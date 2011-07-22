
#include "config.h"
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/StructureEstimator.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include "../src/libmv_mapping.h"
#include "../src/PCL_mapping.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <numeric>

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Only to see how we can create a 3D structure estimation using calibrated cameras
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

#define POINT_METHOD "SIFT"

NEW_TUTO(Triangulation_tuto, "Learn how you can triangulate 2D points",
  "Using fully parameterized cameras, we find 2D points in the sequence and then triangulate them. We finally draw them on the sequence.")
{
  //universal method to get the current image:
  vector<Mat> images;

  SequenceAnalyzer *motion_estim_loaded;
  string pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_"POINT_METHOD"/motion_tracks_triangulated.yml");
  std::ifstream test_file_exist;
  test_file_exist.open( pathFileTracks );
  if( test_file_exist.is_open() )
  {
    test_file_exist.close();
    cout<<"create a SequenceAnalyzer using Medias/tracks_points_"POINT_METHOD"/motion_tracks_triangulated.yml"<<endl;
    FileStorage fsRead(pathFileTracks, FileStorage::READ);
    FileNode myPtt = fsRead.getFirstTopLevelNode();
    motion_estim_loaded = new SequenceAnalyzer( images, myPtt );
    fsRead.release();

    cout<<"numbers of correct tracks loaded:"<<
      motion_estim_loaded->getTracks().size()<<endl;
  }
  else
  {
    pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_"POINT_METHOD"/motion_tracks.yml");
    test_file_exist.open(pathFileTracks);
    if( !test_file_exist.is_open() )
    {
      cout<<"please compute points matches using testMotionEstimator.cpp first!"<<endl;
      return;
    }
    test_file_exist.close();
  
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

    cout<<"Finally create a new SequenceAnalyzer using Medias/tracks_points_"POINT_METHOD"/motion_tracks.yml"<<endl;
    FileStorage fsRead(pathFileTracks, FileStorage::READ);
    FileNode myPtt = fsRead.getFirstTopLevelNode();
    motion_estim_loaded = new SequenceAnalyzer( images, myPtt );
    fsRead.release();

    cout<<"numbers of correct tracks loaded:"<<
      motion_estim_loaded->getTracks().size()<<endl;

    int maxImg=motion_estim_loaded->getNumViews();

    cout<<"triangulation of points."<<endl;
    StructureEstimator structure ( *motion_estim_loaded, myCameras );
    vector<char> mask =  structure.computeStructure();
    cout<<std::accumulate( mask.begin(), mask.end(), 0 )<<" 3D points found."<<endl;

    //now save the triangulate tracks:
    pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_"POINT_METHOD"/motion_tracks_triangulated.yml");
    FileStorage fsOutMotion(pathFileTracks, FileStorage::WRITE);
    if( !fsOutMotion.isOpened() )
      CV_Error(0,"Can't create the file!\nPlease verify you have access to the directory!");

    SequenceAnalyzer::write( fsOutMotion, *motion_estim_loaded );
    fsOutMotion.release();
  }

  vector<Vec3d>& tracks = motion_estim_loaded->get3DStructure();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  OpencvSfM::mapping::convert_OpenCV_vector( tracks, *basic_cloud_ptr );
  
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (basic_cloud_ptr);
  while (!viewer.wasStopped ())
  {
  }
}
