
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

  std::vector<TrackOfPoints> StructureEstimator::computeStructure(
    const vector<int>& list_of_images)
  {
    CV_Assert( list_of_images.size() > 1 );

    std::vector<TrackOfPoints> points3D;
    vector<TrackOfPoints>& tracks = sequence_.getTracks();
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints();

    //for each points:
    vector<TrackOfPoints>::size_type key_size = tracks.size();
    vector<PointOfView>::size_type num_camera = cameras_.size();
    int idImage=-1, idPoint=-1;
    vector<TrackOfPoints>::size_type i;
    vector<int>::size_type images_size =list_of_images.size();

    for (i=0; i < key_size; i++)
    {
      TrackOfPoints &track = tracks[i];
      int nbLinks = 0;
      for( size_t it_img = 0; it_img<images_size ; ++it_img)
        if( track.containImage( list_of_images[ it_img ] ) )
          nbLinks++;

      if( nbLinks > 1 )
      {
        unsigned int nviews = track.getNbTrack();
        vector<bool> mask;
        for(unsigned int j=0; j<nviews; ++j)
        {
          if( std::find(list_of_images.begin(),list_of_images.end(),
            track.getImageIndex( j ) ) != list_of_images.end() )
            mask.push_back(true);
          else
            mask.push_back(false);
        }

        cv::Vec3d point_final;
        double distance=track.triangulateRobust( cameras_,points_to_track, point_final,
          4, mask );

        //if( distance<max_repro_error_ )
          points3D.push_back(track);
      }

    }
    return points3D;
  }

}
