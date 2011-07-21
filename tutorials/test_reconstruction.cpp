
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
  vector<TrackOfPoints> tracks;
  structure.computeStructure(tracks);
  cout<<tracks.size()<<" points found."<<endl;

  vector<Vec3f> points3D;
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  vector<TrackOfPoints>::iterator itTrack=tracks.begin();
  index_image=0;
  while ( itTrack != tracks.end() )
  {
    Vec3f p_tmp = (Vec3f)(*itTrack);
    points3D.push_back( p_tmp );
    pcl::PointXYZ p;
    p.x = p_tmp[0]; p.y = p_tmp[1]; p.z = p_tmp[2];
    basic_cloud_ptr->points.push_back( p );
    itTrack++;
  }
  
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (basic_cloud_ptr);
  while (!viewer.wasStopped ())
  {
  }
}
