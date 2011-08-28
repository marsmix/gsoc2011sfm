#ifndef _GSOC_SFM_CAMERA_PINHOLE_DISTORTION_H
#define _GSOC_SFM_CAMERA_PINHOLE_DISTORTION_H 1

//A lot of methods are inspired of LIBMV project ( http://code.google.com/p/libmv_core/ )

#include "macro.h" //SFM_EXPORTS
#include "CameraPinhole.h"
#include "opencv2/imgproc/imgproc.hpp"

namespace OpencvSfM{
  enum paramsOfDistoParameters{
    RADIAL_PARAM=0x10,  ///<Mask used to know if the radial parameters are estimated or not
    TANGEANT_PARAM=0x20  ///<Mask used to know if the tangeant parameters are estimated or not
  };

  
  /*! \brief This class represent the physical device which take the pictures. 
  *      It is not related to a 3D position which is the role of the PointOfView class.
  *      The role of the class is to store intra parameters and radial distortion
  *
  *
  * So this class is devoted to the conversion between 3D points ( using camera coordinate ) and 2D points ( using image coordinate ) using the methods convertFromImageTo3Dray or convertFrom3DToImage
  */
  class SFM_EXPORTS CameraPinholeDistor:public CameraPinhole{
  protected:
    cv::Vec<double, 6> radial_dist_;///<used to store radial dist parameters ( /f$k_1/f$ to /f$k_6/f$ )
    unsigned char nb_radial_params_;///<number of radial dist parameters ( 0, 2, 3 or 6 )
    cv::Vec<double, 2> tangential_dist_;///<used to store tangential dist parameters ( /f$p_1/f$ and /f$p_2/f$ )
    unsigned char nb_tangent_params_;///<N umbers of tangeancial distorition parameters (0, 1 or 2)
    cv::Mat distortionVector;///<vector of distortion coefficients ( k_1, k_2, p_1, p_2[ , k_3[ , k_4, k_5, k_6 ]] ) of 4, 5 or 8 elements
 
  public:
    /**
    * Constructor with ( or not ) intra parameters.
    * @param intra_params matrix of intra parameters ( 3*3 )
    * @param radial_dist radial dist parameters ( /f$k_1/f$ to /f$k_6/f$ )
    * @param nbRadialParam number of radial dist parameters ( 0, 2, 3 or 6 )
    * @param tangential_dist tangential dist parameters ( /f$p_1/f$ and /f$p_2/f$ )
    * @param wantedEstimation values which need an estimation
    */
    CameraPinholeDistor( cv::Mat intra_params=cv::Mat::eye( 3, 3, CV_64F ),
      cv::Vec6d radial_dist=cv::Vec6d( 0.0,0.0,0.0,0.0,0.0,0.0 ),
      unsigned char nbRadialParam=6,cv::Vec2d tangential_dist=cv::Vec2d( 0.0,0.0 ),
      int img_w=640, int img_h=480,
      unsigned char wantedEstimation=FOCAL_PARAM|SKEW_PARAM|PRINCIPAL_POINT_PARAM|RADIAL_PARAM|TANGEANT_PARAM );
    /**
    * Constructor where initial camera matrix is computed from the 3D-2D point correspondences.
    * Currently, the function only supports planar calibration patterns, i.e. patterns where each object point has z-coordinate =0.
    * @param objectPoints The vector of vectors of the object points. See http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#cv-calibratecamera
    * @param imagePoints The vector of vectors of the corresponding image points. See http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#cv-calibratecamera
    * @param imageSize The image size in pixels; used to initialize the principal point
    * @param aspectRatio If it is zero or negative, both \f$f_x\f$  and \f$f_y\f$  are estimated independently. Otherwise \f$f_x=f_y * aspectRatio\f$ 
    * @param radial_dist radial dist parameters ( /f$k_1/f$ to /f$k_6/f$ )
    * @param nbRadialParam number of radial dist parameters ( 0, 2, 3 or 6 )
    * @param tangential_dist tangential dist parameters ( /f$p_1/f$ and /f$p_2/f$ )
    * @param wantedEstimation values which need an estimation
    */
    CameraPinholeDistor( const std::vector<std::vector<cv::Point3f> >& objectPoints,
      const std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Size imageSize,double aspectRatio=1.,
      cv::Vec6d radial_dist=cv::Vec6d( 0.0,0.0,0.0,0.0,0.0,0.0 ),
      unsigned char nbRadialParam=6,
      cv::Vec2d tangential_dist=cv::Vec2d( 0.0,0.0 ),
      int img_w=640, int img_h=480,
      unsigned char wantedEstimation=FOCAL_PARAM|SKEW_PARAM|PRINCIPAL_POINT_PARAM|RADIAL_PARAM|TANGEANT_PARAM );
    ~CameraPinholeDistor( );

    /**
    * this method can be used to update the intra parameters.
    * @param radial_dist values of the new radial distortions parameters
    * @param nbRadialParam number of radial dist parameters ( 0, 2, 3 or 6 )
    * @param tangential_dist values of the new tangential distortions parameters
    * @param wantedEstimation values which need an estimation
    */
    void updateDistortionParameters( const cv::Vec6d& radial_dist, unsigned char nbRadialParam,const cv::Vec2d& tangential_dist,
      unsigned char wantedEstimation=RADIAL_PARAM|TANGEANT_PARAM );

    /**
    * This method can transform points from image to 3D rays
    */
    virtual std::vector<cv::Vec4d> convertFromImageTo3Dray( std::vector<cv::Vec3d> points );
    
    /**
    * This method can convert 2D points from pixel image coordinates to 2D points in normalized image coordinates
    * @param points 2D points in pixel image homogeneous coordinates.
    * @return 2D points in normalized image homogeneous coordinates.
    */
    virtual std::vector<cv::Vec2d> pixelToNormImageCoordinates( std::vector<cv::Vec2d> points ) const;
    /**
    * This method can convert 2D points from normalized image coordinates to 2D points in pixel image coordinates
    * @param points 2D points in normalized image homogeneous coordinates.
    * @return 2D points in pixel image coordinates.
    */
    virtual std::vector<cv::Vec2d> normImageToPixelCoordinates( std::vector<cv::Vec2d> points ) const;
    
    /**
    * Create a new camera from a YAML file.
    * @param node Previously opened YAML file node
    */
    static cv::Ptr<Camera> read( const cv::FileNode& node );
    
    /**
    * Save the camera intra parameters into a YAML file.
    * @param fs Previously opened YAML file node
    */
    virtual void write( cv::FileStorage& fs ) const;

  };

}

#endif