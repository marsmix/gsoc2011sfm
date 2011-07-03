
#include "StructureEstimator.h"


namespace OpencvSfM{
  using std::vector;
  using cv::Ptr;

  void StructureEstimator::computeStructure(vector<TrackPoints>& points3D)
  {
    vector<TrackPoints>& tracks = sequence_.getTracks();
    vector<Ptr<PointsToTrack>> &points_to_track = sequence_.getPoints();

    //for each points:
    vector<TrackPoints>::size_type key_size = tracks.size();
    vector<PointOfView>::size_type num_camera = cameras_.size();
    int idImage=-1, idPoint=-1;
    vector<TrackPoints>::size_type i;

    for (i=0; i < key_size; i++)
    {
      TrackPoints &track = tracks[i];
      unsigned int nviews = track.getNbTrack();

      CV_Assert(nviews < cameras_.size());

      cv::Vec3d point_final;
      double distance=track.triangulateRobust( cameras_,points_to_track, point_final );
      //double distance=track.triangulateLinear( cameras_,points_to_track, point_final );

      //this is used to take only correct 3D points:
      if(distance<max_repro_error_)
        points3D.push_back(track);

    }
  }
  void StructureEstimator::computeTwoView(int img1, int img2,
    vector<TrackPoints>& points3D)
  {
    vector<TrackPoints>& tracks = sequence_.getTracks();
    vector<Ptr<PointsToTrack>> &points_to_track = sequence_.getPoints();

    //for each points:
    vector<TrackPoints>::size_type key_size = tracks.size();
    vector<PointOfView>::size_type num_camera = cameras_.size();
    int idImage=-1, idPoint=-1;
    vector<TrackPoints>::size_type i;

    for (i=0; i < key_size; i++)
    {
      TrackPoints &track = tracks[i];
      if( track.containImage(img1) && track.containImage(img2) )
      {
        unsigned int nviews = track.getNbTrack();
        vector<bool> mask;
        for(unsigned int j=0; j<nviews; ++j)
        {
          if( img1 == track.getImageIndex( j ) ||
            img2 == track.getImageIndex( j ) )
            mask.push_back(true);
          else
            mask.push_back(false);
        }

        cv::Vec3d point_final;
        double distance=track.triangulateLinear( cameras_,points_to_track, point_final,
          mask );

        points3D.push_back(track);
      }

    }
  }
}