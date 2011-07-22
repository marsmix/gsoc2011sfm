
#include "StructureEstimator.h"


namespace OpencvSfM{
  using std::vector;
  using cv::Ptr;

  vector<char> StructureEstimator::computeStructure()
  {
    vector<char> output_mask;
    vector<TrackOfPoints>& tracks = sequence_.getTracks();
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints();

    //for each points:
    vector<TrackOfPoints>::size_type key_size = tracks.size();
    vector<PointOfView>::size_type num_camera = cameras_.size();
    int idImage=-1, idPoint=-1;
    vector<TrackOfPoints>::size_type i;

    for (i=0; i < key_size; i++)
    {
      TrackOfPoints &track = tracks[i];
      unsigned int nviews = track.getNbTrack();

      CV_DbgAssert(nviews <= cameras_.size());

      cv::Vec3d point_final;
      double distance=track.triangulateRobust( cameras_,points_to_track, point_final );
      //double distance=track.triangulateLinear( cameras_,points_to_track, point_final );

      //this is used to take only correct 3D points:
      output_mask.push_back( (distance<max_repro_error_) );
    }
    return output_mask;
  }
  void StructureEstimator::computeTwoView(int img1, int img2,
    vector<TrackOfPoints>& points3D)
  {
    vector<TrackOfPoints>& tracks = sequence_.getTracks();
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints();

    //for each points:
    vector<TrackOfPoints>::size_type key_size = tracks.size();
    vector<PointOfView>::size_type num_camera = cameras_.size();
    int idImage=-1, idPoint=-1;
    vector<TrackOfPoints>::size_type i;

    for (i=0; i < key_size; i++)
    {
      TrackOfPoints &track = tracks[i];
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
