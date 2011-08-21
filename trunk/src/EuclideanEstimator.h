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
    libmv::vector<libmv::Mat3> intra_params_;
    libmv::vector<libmv::Mat3> rotations_;
    libmv::vector<libmv::Vec3> translations_;
    std::vector<PointOfView>& cameras_;
    SequenceAnalyzer &sequence_;
  public:
    EuclideanEstimator( SequenceAnalyzer &sequence,
      std::vector<PointOfView>& cameras );
    virtual ~EuclideanEstimator( void );

    std::vector< TrackOfPoints > point_computed_;
    std::vector<bool> camera_computed_;

    /**
    * Add a new camera to the estimator
    * @param camera new point of view to add for reconstruction
    */
    void addNewPointOfView( const PointOfView& camera );

    /**
    * comptue cameras and structure if intra parameters are known.
    */
    void computeReconstruction( );

    void bundleAdjustement( );

    void viewEstimation();

    void initialReconstruction( std::vector<TrackOfPoints>& tracks,
      int image1, int image2 );

    bool cameraResection( unsigned int image );
  };

}

#endif
