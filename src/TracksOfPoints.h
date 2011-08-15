
#ifndef _GSOC_SFM_TRACKS_OF_POINTS_H
#define _GSOC_SFM_TRACKS_OF_POINTS_H 1

#include "macro.h" //SFM_EXPORTS

#include <numeric>
#include "opencv2/calib3d/calib3d.hpp"

namespace OpencvSfM{
  class SFM_EXPORTS PointsToTrack;
  class SFM_EXPORTS PointOfView;

  /**
  * \brief This class store the track of keypoints.
  * A track is a connected set of matching keypoints across multiple images
  *
  * This class can be used as a Vec3d because it's the projection of a 3D points
  * Of course, use triangulate method before to create this 3D point!
  *
  * Discussion: Store index of points or 2D position?
  */
  class SFM_EXPORTS TrackOfPoints
  {
    friend class SequenceAnalyzer;

  protected:
    cv::Ptr<cv::Vec3d> point3D;
    std::vector<unsigned int> images_indexes_;
    std::vector<unsigned int> point_indexes_;
    unsigned int color;
    /**
    * Sometimes a 2d point is not good...
    * This vector help us to know which points are correct...
    */
    std::vector<bool> good_values;
    /**
    * if <0 the track is inconsistent
    * if >0 represent the degree of consistence ( higher is better )
    */
    int track_consistance;
  public:
    /**
    * cast operator to use this object as a 3D point!
    */
    template<typename Type, int size>
    operator cv::Vec<Type,size>&( ) {
      return *point3D;
    }
    TrackOfPoints( ):track_consistance( 0 ),color(0){};
    ~TrackOfPoints();
    /**
    * This function add matches to track
    * @param image_src index of source matches image
    * @param point_idx index of point in source image
    * @return true if this match is correct, false if inconsistent with
    * Snavely's rules.
    */
    bool addMatch( const int image_src, const int point_idx );

    /**
    * This function is used to know if the track contains the image
    * @param image_wanted index of query image
    * @return true if this track contain points from the query image
    */
    inline bool containImage( const int image_wanted ) const
    {
      std::vector<unsigned int>::const_iterator idx = std::find(
        images_indexes_.begin( ),images_indexes_.end( ),
        image_wanted );
      return (idx != images_indexes_.end( ) &&
        good_values[idx - images_indexes_.begin( ) ]==true );
    }
    /**
    * This function is used to know if the track contains the query point
    * @param image_src index of query image
    * @param point_idx1 index of point in query image
    * @return true if this track contain the point from the query image
    */
    bool containPoint( const int image_src, const int point_idx1 );
    /**
    * This function is used to get the numbers of image for this track
    * @return 0 if inconsistent, >= 2 else
    */
    inline unsigned int getNbTrack( ) const
    {
      return track_consistance<0 ? 0 : std::accumulate(
    good_values.begin( ), good_values.end( ), 0 );
    };
    /**
    * use this function to create a DMatch value from this track
    * @param img1 train match image
    * @param img2 query match image
    * @return DMatch value
    */
    cv::DMatch toDMatch( const int img1,const int img2 ) const;
    /**
    * use this function to get the n^th match value from this track
    * @param index which match
    * @param idImage out value of the image index
    * @param idPoint out value of the point index
    */
    void getMatch( const unsigned int index,
      int &idImage, int &idPoint ) const;
    /**
    * use this function to get the index point of the wanted image
    * @param image index of wanted image
    * @return index of point
    */
    inline int getPointIndex( const unsigned int image ) const
    {
      std::vector<unsigned int>::const_iterator result =
        std::find( images_indexes_.begin( ),images_indexes_.end( ),image );
      if( result == images_indexes_.end( ) )
        return -1;
      return point_indexes_[ result - images_indexes_.begin( ) ];
    };
    /**
    * use this function to get the image corresponding to the nth entry
    * of this track
    * @param idx index of wanted point
    * @return number of image
    */
    inline int getImageIndex( const unsigned int idx ) const
    {
      return images_indexes_[ idx ];
    };

    double triangulateLinear( std::vector<PointOfView>& cameras,
      const std::vector< cv::Ptr<PointsToTrack> > &points_to_track,
      cv::Vec3d& points3D,
      const std::vector<bool> &masks = std::vector<bool>( ) );
    double triangulateRobust( std::vector<PointOfView>& cameras,
      const std::vector< cv::Ptr< PointsToTrack > > &points_to_track,
      cv::Vec3d& points3D,
      double reproj_error = 4,
      const std::vector<bool> &masks = std::vector<bool>( ) );

    void removeOutliers( std::vector<PointOfView>& cameras,
      const std::vector< cv::Ptr< PointsToTrack > > &points_to_track,
      double reproj_error = 4,
      std::vector<bool> *masks = NULL );


    inline void set3DPosition( cv::Vec3d newPoint )
    {
      if( !point3D )
        point3D = new cv::Vec3d( newPoint );
      else
        *point3D = newPoint;
    }
    inline cv::Ptr<cv::Vec3d> get3DPosition(){ return point3D; };

    inline unsigned int getColor() const {return color;};
    inline void setColor(unsigned int c) {color = c;};

    static void keepTrackHavingImage( unsigned int idx_image,
      std::vector<TrackOfPoints>& tracks );
    static void keepTrackWithImages( const
      std::vector<int>& imgList,
      std::vector<TrackOfPoints>& tracks );
  protected:
    double errorEstimate( std::vector< PointOfView >& cameras,
      const std::vector< cv::Ptr< PointsToTrack > > &points_to_track,
      cv::Vec3d& points3D,
      const std::vector<bool> &masks = std::vector<bool>( ) ) const;
  };


  /**
  * @brief This structure store an image link ( two image ids )...
  */
  typedef struct
  {
    int imgSrc;
    int imgDest;
  } ImageLink;
  /**
  * \brief This class modelizes the images graph connections
  *
  */
  class SFM_EXPORTS ImagesGraphConnection
  {
  protected:
    /**
    * Sparse upper triangular matrix for image graph.
    * ( i,j ) value represent the numbers of points matches between image i and j.
    * of course ( i,j ) equal ( j,i ) so only ( i,j ) with i<j are stored.
    */
    cv::SparseMat images_graph_;

    inline void orderedIdx( int i1,int i2,int idx[ 2 ] )
    {
      if( i1<i2 )
      {
        idx[ 0 ] = i1;
        idx[ 1 ] = i2;
      }
      else
      {
        idx[ 0 ] = i2;
        idx[ 1 ] = i1;
      }
    }
  public:
    ImagesGraphConnection( ){};

    /**
    * Use this function to test if the graph is already builded
    * @param nbImages number of images the graph should store
    * @return true if graph is build
    */
    inline bool isGraphCreated( int nbImages )
    {
      return images_graph_.size( )!=0 && images_graph_.nzcount( )!=0 &&
        images_graph_.size( )[ 0 ] == nbImages &&
        images_graph_.size( )[ 1 ] == nbImages;
    }

    /**
    * Prepare this structure to store the graph of correspondances
    * @param nb_images number of images to store
    */
    inline void initStructure( int nb_images )
    {
      const int dims = 2;
      int size[ ] = {nb_images,nb_images};
      images_graph_.create( dims,size,CV_32S );
    }
    /**
    * Add a new link between two images
    * @param first_image first image
    * @param second_image second image
    */
    inline void addLink( int first_image,int second_image )
    {
      int idx[ 2 ];
      orderedIdx( first_image,second_image,idx );
      images_graph_.ref<int>( idx ) += 1;
    }
    /**
    * get the numbers of links between two images
    * @param first_image first image
    * @param second_image second image
    * @return numbers of links between first image and second image
    */
    inline int getNumbersOfLinks( int first_image,int second_image )
    {
      int idx[ 2 ];
      orderedIdx( first_image,second_image,idx );
      const int *value=images_graph_.find<int>( idx );
      if( value!=NULL )
        return *value;
      return 0;
    }
    /**
    * get the highest link
    * @param first_image [ out ] first image
    * @param second_image [ out ] second image
    * @param max_number [ in ] maximum allowed links between images
    * @return numbers of links between first image and second image
    */
    int getHighestLink( int &first_image,int &second_image,
      int max_number=1e9 );
    /**
    * get the highest link
    * @param outList [ out ] ordered vector of links between images
    * @param min_number minimum allowed links between images
    * @param max_number maximum allowed links between images
    */
    void getOrderedLinks( std::vector<ImageLink>& outList,
      int min_number=0, int max_number=1e9 );
    /**
    * get the related images to the first parameter
    * @param first_image [ in ] first image index
    * @param outList [ in/out ] ordered vector of links between images
    * @param min_number minimum allowed links between images
    * @param max_number maximum allowed links between images
    */
    void getImagesRelatedTo( int first_image, std::vector<ImageLink>& outList,
      int min_number=0, int max_number=1e9 );
  };
}
#endif
