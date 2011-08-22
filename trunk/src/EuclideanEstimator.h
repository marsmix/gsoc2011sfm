#ifndef _GSOC_SFM_POINTS_3D_EUCLIDEAN_ESTIMATION_H
#define _GSOC_SFM_POINTS_3D_EUCLIDEAN_ESTIMATION_H 1

#include <iostream>
#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"
#include "opencv2/core/eigen.hpp"

#include "macro.h" //SFM_EXPORTS
#include "SequenceAnalyzer.h"
#include "PointOfView.h"

namespace OpencvSfM{

  /** \brief This class perform a projective estimation
  * of the motion. Given points matches and cameras with intra parameters,
  * it tries to find the best cameras positions and 3D points. Does not
  * perform a bundle ajustement!
  *
  * As this class use a lot of libmv functions, the data members are
  * using libmv structures...
  */
  class SFM_EXPORTS EuclideanEstimator
  {
  protected:
    int index_origin;///<index of camera set as origin...
    libmv::vector<libmv::Mat3> intra_params_;///<Intra parameters of cameras (don't use them, they are strongly related to cameras_ attribut!
    libmv::vector<libmv::Mat3> rotations_;///<rotations matrix of cameras (don't use them, they are strongly related to cameras_ attribut!
    libmv::vector<libmv::Vec3> translations_;///<translation vectors of cameras (don't use them, they are strongly related to cameras_ attribut!
    std::vector<PointOfView>& cameras_;///<List of cameras (intra and extern parameters...)
    SequenceAnalyzer &sequence_;///<Object containing all 2D information of this sequence
  public:
    /**
    * Construct an euclidean estimator using a sequence of 2D points matches and
    * a list of camera guess (intra parameters should be known!)
    * @param sequence Object containing all 2D information of this sequence
    * @param cameras List of cameras (intra (and extern if available) parameters...)
    */
    EuclideanEstimator( SequenceAnalyzer &sequence,
      std::vector<PointOfView>& cameras );
    /**
    * Destructor of EuclideanEstimator
    */
    virtual ~EuclideanEstimator( void );

    std::vector< TrackOfPoints > point_computed_;///<list of 3D points computed
    std::vector<bool> camera_computed_;///<List of camera computed

    /**
    * Add a new camera to the estimator
    * @param camera new point of view to add for reconstruction
    */
    void addNewPointOfView( const PointOfView& camera );

    /**
    * comptue cameras and structure if intra parameters are known.
    */
    void computeReconstruction( );

    /**
    * Run a bundle adjustment using every computed cameras and every computed 3D points
    */
    void bundleAdjustement( );

    /**
    * Show this estimation
    */
    void viewEstimation();
    
    /**
    * Create a new Euclidean reconstruction using matches between two images
    * @param image1 index of the first image
    * @param image2 index of the second image
    */
    void initialReconstruction( int image1, int image2 );
    
    /**
    * Find the position of a new camera
    * @param image index of the wanted camera
    */
    bool cameraResection( unsigned int image, int max_reprojection = 50 );

    /**
    * Find matches between img1 and img2 and add the to the reconstruction...
    * @param img1 index of the first image
    * @param img2 index of the second image
    */
    void addMoreMatches(int img1, int img2);
  };

}

#endif
