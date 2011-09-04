#ifndef _GSOC_SFM_POINTS_TO_TRACK_H
#define _GSOC_SFM_POINTS_TO_TRACK_H 1

#include "macro.h" //SFM_EXPORTS and remove annoying warnings

#include <vector>
#include "opencv2/features2d/features2d.hpp"

#include "config_SFM.h"//semaphore...


namespace OpencvSfM{
  class SFM_EXPORTS TrackOfPoints;

/*! \brief This class can be used to store informations about point
  * and features. This is an abstract class: you can't use it directly.
  * Use for instance PointsToTrackWithImage.
  *
  * To create a structure from motion, most methods use points to compute
  * the structure. This class focus on the first task in SfM: find points
  * in image which are easy to track...
  * When available, a feature vector for each points is very helpful:
  * the matching will be easier.
  */
  class SFM_EXPORTS PointsToTrack
  {
  protected:
    /**
    * As we want to be able to compute points using parallel execution,
    * and as not every Opencv functions are thread safe,
    * use this mutex to take care of critical portions.
    */
    DECLARE_MUTEX(worker_exclusion);
    /**
    * To preserve memory, we need to know how many process
    * are working with theses points...
    */
    unsigned int nb_workers_;
    /**
    * This attribute will store points coordinates
    * and sometimes orientation and size
    */
    std::vector<cv::KeyPoint> keypoints_;
    /**
    * this attribute will store descritors for each points in a matrix
    * with size ( n*m ), where
    * n is the number of points and m is the desciptor size.
    */
    cv::Mat descriptors_;

    /**
    * When available, the picture from where points are detected
    */
    cv::Mat imageToAnalyse_;
    /**
    * When available, the color of each point can be stored here.
    */
    std::vector<unsigned int> RGB_values_;
    int corresponding_image_;///<index of frame when available
    static int glob_number_images_;///<total numbers of images!

    /**
    * This is the method you should implement when you create a new
    * point detector algorithm.
    * @return the number of points
    */
    virtual int impl_computeKeypoints_( ){return 0;};
    /**
    * This is the method you should implement when you create a new
    * descriptors extractor...
    */
    virtual void impl_computeDescriptors_( ){};

  public:
    /**
    * this constructor create an object with available information...
    * @param corresponding_image Global index of image
    * @param keypoints the points we will try to track...
    * @param descriptors the feature vector for each points...
    */
    PointsToTrack( int corresponding_image=-1, std::vector<cv::KeyPoint> keypoints=
      std::vector<cv::KeyPoint>( 0 ), cv::Mat descriptors=cv::Mat( ) );
    /**
    * Destructor : delete points and features vectors
    */
    virtual ~PointsToTrack( void );
    /**
    * To preserve memory, we use this method to free descriptors
    * @param force if true, the descriptors are removed
    */
    void free_descriptors( bool force = false );

    /**
    * This method is used to compute both Keypoints and descriptors...
    * @param forcing_recalculation if true previous keypoints are removed...
    * If false and if keypoints and descriptor exists, nothing is done.
    * @return the number of points
    */
    int computeKeypointsAndDesc( bool forcing_recalculation=false );
    /**
    * This method is used to compute only Keypoints...
    * @return the number of points
    */
    int computeKeypoints( );
    /**
    * This method is used to compute only descriptors...
    */
    void computeDescriptors( );
    /**
    * This method is used to add Keypoints...
    * @param keypoints Keypoints to add
    * @param descriptors of points, if any
    * @param computeMissingDescriptor if true, the missing descriptors are computed.
    */
    void addKeypoints( std::vector<cv::KeyPoint> keypoints,cv::Mat descriptors=cv::Mat( ),bool computeMissingDescriptor=false );
    /**
    * This method is used to add a keypoint at the end of the points vector...
    * @param point Keypoints to add
    * @return index of the keypoint.
    */
    inline unsigned int addKeypoint( const cv::KeyPoint point )
    {
      P_MUTEX(worker_exclusion);
      keypoints_.push_back( point );
      V_MUTEX(worker_exclusion);
      return keypoints_.size( )-1;
    };
    /**
    * this method return the points coordinates and sometimes orientation and size
    * @return points coordinates and when available orientation and size
    */
    inline std::vector<cv::KeyPoint>& getModifiableKeypoints( ) {return keypoints_;};
    /**
    * this method return the points coordinates and sometimes orientation and size
    * @return points coordinates and when available orientation and size
    */
    inline const std::vector<cv::KeyPoint>& getKeypoints( ) const {return keypoints_;};
    /**
    * This method update the points coordinates (last parameter) corresponding
    * to tracks containing image index "otherImage"
    * @param matches list of tracks. Only points found in tracks are returned
    * @param otherImage index of wanted image
    * @param pointsVals [ out ] points found in tracks
    */
    void getKeyMatches( const std::vector<TrackOfPoints>& matches, int otherImage,
      std::vector<cv::Point2f>& pointsVals ) const;
    /**
    * this method return the points coordinates of the i^th entry
    * @param index number of keypoints wanted
    * @return points coordinates and when available orientation and size
    */
    inline const cv::KeyPoint& getKeypoint( unsigned int index ) const
    {
      CV_DbgAssert( index<keypoints_.size( ) );
      return keypoints_[ index ];
    };
    /**
    * this method return the closest points from parameter
    * @param point coordinate of the point to search for
    * @return index of the point
    */
    size_t getClosestKeypoint( cv::Point2f point );
    /**
    * this method return the descritors for each points in a matrix with size ( n*m ), where n is the number of points and m is the desciptor size.
    * @return descritors for each points in a matrix with size ( n*m ), where n is the number of points and m is the desciptor size.
    */
    cv::Mat getDescriptors( ) const {return descriptors_;};
    /**
    * Get the image used to compute points
    */
    inline cv::Mat getImage( ){return imageToAnalyse_;};
    /**
    * To show the points on image, use this function to draw points on it.
    * @param image Source image.
    * @param outImg Output image. Its content depends on flags value what is drawn in output image. See possible flags bit values.
    * @param color Color of keypoints
    * @param flags Possible flags bit values is defined by DrawMatchesFlags ( see http://opencv.willowgarage.com/documentation/cpp/features2d_drawing_function_of_keypoints_and_matches.html#cv-drawmatches )
    */
    //
    void printPointsOnImage( const cv::Mat &image, cv::Mat& outImg, const cv::Scalar& color=cv::Scalar::all( -1 ), int flags=cv::DrawMatchesFlags::DEFAULT ) const;

    /**
    * Use this function to get the color of a point
    * @param index of the wanted point
    * @return color packed into the ARGB format
    */
    inline unsigned int getColor( unsigned int index ) const{
      if( index<RGB_values_.size() )
        return RGB_values_[index];
      else
        return 0;
    }

    /**
    * Load the points from a YAML file.
    * @param node Previously opened YAML file node
    * @param points output
    */
    static void read( const cv::FileNode& node, PointsToTrack& points );

    /**
    * Save the points into a YAML file.
    * @param fs Previously opened YAML file node
    * @param points sequence to save...
    */
    static void write( cv::FileStorage& fs, const PointsToTrack& points );
  };
}
/*
//! writes vector of keypoints to the file storage
static inline void cv::write( cv::FileStorage& fs, const std::string& name,
  const OpencvSfM::PointsToTrack& points )
{
  OpencvSfM::PointsToTrack::write( fs,points );
};
//! reads vector of keypoints from the specified file storage node
static inline void cv::read( const cv::FileNode& node, OpencvSfM::PointsToTrack& points,
  OpencvSfM::PointsToTrack defaultValue = OpencvSfM::PointsToTrack( ) )
{
  OpencvSfM::PointsToTrack::read( node,points );
}
*/
#endif
