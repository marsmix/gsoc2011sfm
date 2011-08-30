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
  struct MatchingThread;

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
    friend struct MatchingThread;
  protected:
    static int mininum_points_matches;///<Minimum points detected into an image to keep this estimation (set to 20)
    static int mininum_image_matches;///<Minimum images connections in a track to keep this estimation (usually set to 2)
    /**
    * optional, method to use for feature detection
    */
    cv::Ptr<cv::FeatureDetector> feature_detector_;
    /**
    * optional, method to use for feature extraction
    */
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;
    /**
    * A list of points for each picture
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
    /**
    * A list of the fundamental matrix between each points*
    * this list can have some NULL values as the fundamental matrix
    * don't exist between every images...
    * This matrix is uper-triangular, that is [2][1] exist, but [1][2] not...
    */
    std::vector< std::vector< cv::Ptr< cv::Mat > > > list_fundamental_;
  public:
    /**
    * Constructor taking a MotionProcessor to load images and a features detector
    * and descriptor to find matches.
    * @param input_sequence input images
    * @param feature_detector Algorithm to use for features detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#featuredetector )
    * @param descriptor_extractor Algorithm to use for descriptors detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#descriptorextractor )
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
      std::vector<cv::Mat> *images = NULL,
      cv::Ptr<PointsMatcher> match_algorithm = NULL );
    /**
    * Constructor taking a list of images and a FileNode
    * @param images input images. Points should be in the same order!
    * @param file YAML file to get points and matches
    */
    SequenceAnalyzer( cv::FileNode file,
      std::vector<cv::Mat> *images = NULL,
      cv::Ptr<PointsMatcher> match_algorithm = NULL );

    /**
    * Destructor of SequenceAnalyzer (nothing is released!)
    */
    ~SequenceAnalyzer( void );
    /**
    * This method add new image to track. When adding, if the matches are not
    * computed, use automatically computeMatches to compute them!
    * @param image New image
    * @param points extracted points with features vectors.
    */
    void addNewImage( cv::Mat image,
      cv::Ptr<PointsToTrack> points = cv::Ptr<PointsToTrack>( ) );

    /**
    * This method compute the matches between each points of each images.
    * It first compute missing features descriptor, then train each matcher.
    * Finally compute tracks of keypoints ( a track is a connected set of
    * matching keypoints across multiple images )
    * @param  printProgress set to true is you want to view progress.
    */
    void computeMatches( bool printProgress = true );
    /**
    * This method keep only tracks with more than mininum_image_matches
    */
    static void keepOnlyCorrectMatches(
      std::vector<TrackOfPoints>& tracks,
      unsigned int min_matches = 10,
      unsigned int min_consistance = 3);
    /**
    * This method keep only tracks with more than mininum_image_matches
    */
    static inline void keepOnlyCorrectMatches(
      SequenceAnalyzer& tracks,
      unsigned int min_matches = 10,
      unsigned int min_consistance = 3)
      {
        keepOnlyCorrectMatches(tracks.getTracks(),min_matches,min_consistance);
      }
    /**
    * This method can be used to get the tracks
    */
    inline std::vector<TrackOfPoints> &getTracks( ){return tracks_;};
    /**
    * This method can be used to get the points
    */
    inline std::vector< cv::Ptr< PointsToTrack > > &getPoints( ){
      return points_to_track_;};

    /**
    * Get the graph of image connections
    * @return graph of image connections
    */
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
    /**
    * Use this function to print the sequence of matches
    * @param img_to_show index of image whose tracks will be shown.
    * @param timeBetweenImg see cv::waitKey for the value
    */
    void showTracks( int img_to_show, int timeBetweenImg );
    /**
    * Use this function to print the matches between two images
    */
    void showTracksBetween( unsigned int img1, unsigned int img2 );

    /**
    * Load the sequence from a YAML file.
    * @param node Previously opened YAML file node
    * @param points output
    */
    static void read( const cv::FileNode& node, SequenceAnalyzer& points );

    /**
    * Save the sequence into a YAML file.
    * @param fs Previously opened YAML file node
    * @param points sequence to save...
    */
    static void write( cv::FileStorage& fs, const SequenceAnalyzer& points );

    /**
    * Use this function to know how many images are stored into tracks...
    * @return numbers of images (and cameras) stored into tracks.
    */
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

    /**
    * get the ith image. No checks are performed!
    * @param idx index of the wanted image
    * @return Matrix of the wanted image
    */
    inline cv::Mat getImage( int idx ) { return images_[ idx ]; };

    /**
    * This function add matches to tracks
    * @param newMatches new matches to add
    * @param img1 index of source matches image
    * @param img2 index of destination matches image
    */
    void addMatches( std::vector<cv::DMatch> &newMatches,
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
    /**
    * Use this function to show 2D points into ith image
    * @param i index of wanted image
    * @param pixelProjection list of 2D points
    */
    void showPointsOnImage(unsigned int i,
      const std::vector<cv::Vec2d>& pixelProjection);

    /**
    * Get the points for track from this sequence.
    * @return points for track from this sequence.
    */
    inline std::vector< cv::Ptr< PointsToTrack > > getPointsToTrack()
    { return points_to_track_; };
    /**
    * Get a copy of the points Matcher algorithm from this sequence.
    * @return points Matcher algorithm from this sequence.
    */
    inline cv::Ptr<PointsMatcher> getMatchAlgo()
    { return match_algorithm_->clone( true ); };

    /**
    * This will find matches between two points matchers
    * @param point_matcher first image
    * @param point_matcher second image
    * @param mininum_points_matches minimum matches allowed
    */
    static std::vector< cv::DMatch > simple_matching(
      cv::Ptr<PointsMatcher> point_matcher,
      cv::Ptr<PointsMatcher> point_matcher1,
      int mininum_points_matches = 10);
  };

}

#endif
