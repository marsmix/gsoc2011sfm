#ifndef _GSOC_SFM_SEQUENCE_ANALYZER_H
#define _GSOC_SFM_SEQUENCE_ANALYZER_H 1

#include "macro.h" //SFM_EXPORTS
#include "PointsToTrackWithImage.h"
#include "MotionProcessor.h"
#include "PointsMatcher.h"
#include "PointOfView.h"
//#include "libmv_mapping.h"
#include "TracksOfPoints.h"
#include "opencv2/calib3d/calib3d.hpp"

namespace OpencvSfM{


  /**
  * \brief This class tries to match points in the entire sequence.
  * It follow ideas proposed by Noah Snavely:
  * Modeling the World from Internet Photo Collections
  *
  * This class process an input video to first extracts the
  * features, then matches them and keeps them only when
  * there is more than 2 pictures containing the point.
  */
  class SFM_EXPORTS SequenceAnalyzer
  {
  protected:
    static const int mininum_points_matches = 20;
    static const int mininum_image_matches = 3;
    /**
    * optional, method to use for feature detection
    */
    cv::Ptr<cv::FeatureDetector> feature_detector_;
    /**
    * optional, method to use for feature extraction
    */
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;
    /**
    * One list of points for each picture
    */
    std::vector< cv::Ptr< PointsToTrack > > points_to_track_;
    /**
    * List of input images
    */
    std::vector<cv::Mat> images_;
    /**
    * The matcher algorithm we should use to find matches.
    */
    cv::Ptr<PointsMatcher> match_algorithm_;
    /**
    * A matcher for each picture. Its role is to find quickly matches between
    * i^th picture and other images.
    */
    std::vector< cv::Ptr< PointsMatcher > > matches_;
    /**
    * List of each tracks found. A track is a connected set of matching
    * keypoints across multiple images
    */
    std::vector<TrackOfPoints> tracks_;
    /**
    * Graph of images relations ( value ( i,j ) correspond to the numbers
    * of matches between theses two images
    */
    ImagesGraphConnection images_graph_;
  public:
    /**
    * Constructor taking a MotionProcessor to load images and a features detector
    * and descriptor to find matches.
    * @param input_sequence input images
    * @param feature_detector Algorithm to use for features detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#featuredetector )
    * @param descriptor_detector Algorithm to use for descriptors detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#descriptorextractor )
    * @param match_algorithm algorithm to match points of each images
    */
    SequenceAnalyzer( MotionProcessor input_sequence,
      cv::Ptr<cv::FeatureDetector> feature_detector,
      cv::Ptr<cv::DescriptorExtractor> descriptor_extractor,
      cv::Ptr<PointsMatcher> match_algorithm );
    /**
    * Constructor taking a vector of points to track and a PointsMatcher
    * algorithm to find matches.
    * @param images input images. Points should be in the same order!
    * @param points_to_track list of points to track with ( or not ) features
    * @param match_algorithm algorithm to match points of each images
    */
    SequenceAnalyzer(
      std::vector< cv::Ptr< PointsToTrack > > &points_to_track,
      cv::Ptr<PointsMatcher> match_algorithm,
      const std::vector<cv::Mat> &images );
    /**
    * Constructor taking a list of images and a FileNode
    * @param images input images. Points should be in the same order!
    * @param file YAML file to get points and matches
    */
    SequenceAnalyzer( std::vector<cv::Mat> &images, cv::FileNode file );

    ~SequenceAnalyzer( void );
    /**
    * This method add new image to track. When adding, the matches are not
    * computed, use computeMatches to compute them!
    * @param points extracted points with features vectors.
    */
    void addNewImage( cv::Mat image,
      cv::Ptr<PointsToTrack> points = cv::Ptr<PointsToTrack>( ) );

    /**
    * This method compute the matches between each points of each images.
    * It first compute missing features descriptor, then train each matcher.
    * Finally compute tracks of keypoints ( a track is a connected set of
    * matching keypoints across multiple images )
    */
    void computeMatches( );
    /**
    * This method keep only tracks with more than mininum_image_matches
    */
    void keepOnlyCorrectMatches( );
    /**
    * This method can be used to get the tracks
    */
    inline std::vector<TrackOfPoints> &getTracks( ){return tracks_;};
    /**
    * This method can be used to get the points
    */
    inline std::vector< cv::Ptr< PointsToTrack > > &getPoints( ){
      return points_to_track_;};

    inline ImagesGraphConnection& getImgGraph( )
    {
      if( !images_graph_.isGraphCreated( images_.size( ) ) )
        constructImagesGraph( );
      return images_graph_;
    };
    /**
    * Use this function to print the sequence of matches
    * @param timeBetweenImg see cv::waitKey for the value
    */
    void showTracks( int timeBetweenImg=25 );

    static void read( const cv::FileNode& node, SequenceAnalyzer& points );

    static void write( cv::FileStorage& fs, const SequenceAnalyzer& points );

    inline int getNumViews( ) const
    {
      unsigned int maxImg=0;
      std::vector<TrackOfPoints>::size_type key_size = tracks_.size( ),
        i=0;
      for ( i=0; i < key_size; i++ )
      {
        const TrackOfPoints &track = tracks_[ i ];
        int nviews = track.images_indexes_.size( );
        for( int j = 0;j<nviews;++j )
          if( maxImg<track.images_indexes_[ j ] )
            maxImg=track.images_indexes_[ j ];
      }
      return maxImg;
    }

    inline cv::Mat getImage( int idx ) { return images_[ idx ]; };

    /**
    * This function add matches to tracks
    * @param newMatches new matches to add
    * @param img1 index of source matches image
    * @param img2 index of destination matches image
    */
    inline void addMatches( std::vector<cv::DMatch> &newMatches,
      unsigned int img1, unsigned int img2 );
    /**
    * This function add new Tracks
    * @param newTracks new Tracks to add
    */
    void addTracks( std::vector<TrackOfPoints> &newTracks );
    /**
    * This function constructs and feeds the images_graph_
    */
    void constructImagesGraph( );
    /**
    * This function will create a list of points color corresponding to
    * object viewed in the sequence
    */
    std::vector< unsigned int > getColors( );
    /**
    * This function will create a list of 3D points corresponding to
    * object viewed in the sequence
    */
    std::vector< cv::Vec3d > get3DStructure( );
  };

}

#endif
