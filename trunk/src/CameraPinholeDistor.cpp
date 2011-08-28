#include "CameraPinholeDistor.h"

using std::vector;
using cv::Mat;
using cv::Vec2d;
using cv::Vec3d;
using cv::Vec4d;
using cv::Vec6d;

namespace OpencvSfM{

  CameraPinholeDistor::CameraPinholeDistor( cv::Mat intra_params,
    cv::Vec6d radial_dist, unsigned char nbRadialParam,
    cv::Vec2d tangential_dist, int img_w, int img_h,
    unsigned char wantedEstimation )
    :CameraPinhole( intra_params, img_w, img_h, wantedEstimation )
  {
    updateDistortionParameters( radial_dist,nbRadialParam,tangential_dist );
  }

  CameraPinholeDistor::CameraPinholeDistor( 
    const std::vector< std::vector< cv::Point3f> >& objectPoints,
    const std::vector< std::vector< cv::Point2f> >& imagePoints,
    cv::Size imageSize, double aspectRatio/*=1.*/,
    cv::Vec6d radial_dist, unsigned char nbRadialParam,
    cv::Vec2d tangential_dist, int img_w, int img_h,
    unsigned char wantedEstimation )
    :CameraPinhole( objectPoints,imagePoints,imageSize,aspectRatio,
     img_w, img_h, wantedEstimation )
  {
    updateDistortionParameters( radial_dist,nbRadialParam,tangential_dist );
  }
  CameraPinholeDistor::~CameraPinholeDistor( )
  {
    //TODO
  }

  void CameraPinholeDistor::updateDistortionParameters(
    const cv::Vec6d& radial_dist, unsigned char nbRadialParam,
    const cv::Vec2d& tangential_dist,unsigned char wantedEstimation )
  {
    this->estimation_needed_= ( this->estimation_needed_ & 0x0F ) | ( wantedEstimation & 0xF0 );
    this->radial_dist_=radial_dist;
    this->nb_radial_params_=nbRadialParam;
    this->tangential_dist_=tangential_dist;
    if( estimation_needed_&RADIAL_PARAM )
    {
      int nbParamTotal=nb_radial_params_ + ( (estimation_needed_&TANGEANT_PARAM )>>4 );
      distortionVector.create( nbParamTotal,1,CV_64F );//( k_1, k_2, p_1, p_2[ , k_3[ , k_4, k_5, k_6 ]] )
      double * vals=( double* )distortionVector.data;
      vals[ 0 ]=radial_dist_[ 0 ];
      vals[ 1 ]=radial_dist_[ 1 ];
      if( estimation_needed_&TANGEANT_PARAM )
      {
        vals[ 2 ]=tangential_dist_[ 0 ];
        vals[ 3 ]=tangential_dist_[ 1 ];
        if( nbRadialParam>4 )
        {
          vals[ 4 ]=radial_dist_[ 2 ];
          if( nbRadialParam>5 )
          {
            vals[ 5 ]=radial_dist_[ 3 ];
            vals[ 6 ]=radial_dist_[ 4 ];
            vals[ 7 ]=radial_dist_[ 5 ];
          }
        }
      }
    }
  }

  std::vector< cv::Vec4d > CameraPinholeDistor::convertFromImageTo3Dray( 
    std::vector< cv::Vec3d > points )
  {
    //TODO
    return vector<Vec4d>( );
  }

  vector<Vec2d> CameraPinholeDistor::pixelToNormImageCoordinates(
    std::vector< cv::Vec2d > points ) const
  {
    vector<Vec2d> undistordedPoints;
    //rectify the distorion, but keep points in pixels coordinates:
    cv::undistortPoints( points,undistordedPoints,this->intra_params_,this->distortionVector );
    return undistordedPoints;
  }

  vector<Vec2d> CameraPinholeDistor::normImageToPixelCoordinates( 
    std::vector<cv::Vec2d> points ) const
  {

    vector<Vec2d> redistortedPoints;

    double r2, icdist, deltaX, deltaY;
    double xn, yn, xc, yc;

    for (unsigned int i = 0; i < points.size(); i++) {
        // Extract normalized, undistorted point co-ordinates
        xn = points.at(i)[0];
        yn = points.at(i)[1];

        //printf("%s << (xn, yn) = (%f, %f)\n", __FUNCTION__, xn, yn);

        // Determine radial distance from centre
        r2 = xn*xn + yn*yn;

        // Determine distortion factors (might only work for rational model at this stage)
        icdist = (1 + ((radial_dist_[5]*r2 + radial_dist_[4])*r2 + radial_dist_[3])*r2)/
          (1 + ((radial_dist_[2]*r2 + radial_dist_[1])*r2 + radial_dist_[0])*r2);
        deltaX = 2*tangential_dist_[0]*xn*yn + tangential_dist_[1]*(r2 + 2*xn*xn);
        deltaY = tangential_dist_[0]*(r2 + 2*yn*yn) + 2*tangential_dist_[1]*xn*yn;

        // Distort the points, but keep them in normalized co-ordinate system
        xc = (xn/icdist) + deltaX;
        yc = (yn/icdist) + deltaY;

        redistortedPoints.push_back(cv::Point2d(xc, yc));

        //printf("%s << (xc, yc) = (%f, %f)\n", __FUNCTION__, xc, yc);

    }

    // Convert these re-distorted but normalized points back to pixel co-ordinates
    vector<Vec2d> pointsPixelCoord =
      CameraPinhole::normImageToPixelCoordinates( redistortedPoints );

    return pointsPixelCoord;
  }

  cv::Ptr<Camera> CameraPinholeDistor::read( const cv::FileNode& node )
  {
    std::string myName=node.name( );
    if( myName != "CameraPinholeDistor" )
    {
      std::string error = "CameraPinholeDistor FileNode is not correct!\n"
        "Expected \"CameraPinholeDistor\", got ";
      error += node.name();
      CV_Error( CV_StsError, error.c_str() );
    }

    Mat intra_params;
    Vec6d radial_dist;
    unsigned char nbRadialParam;
    cv::Vec2d tangential_dist;

    //load intra parameters:
    cv::Ptr<Camera> cam_tmp = CameraPinhole::read( node["CameraPinhole"] );

    node[ "nb_radial_params_" ] >> nbRadialParam;
    for(int i=0;i<nbRadialParam;i++)
      radial_dist[i] = node[ "radial_dist_" ][i];
    for(int i=nbRadialParam;i<6;i++)
      radial_dist[i] = 0;

    for(int i=0;i<2;i++)
      tangential_dist[i] = node[ "tangential_dist_" ][i];

    return cv::Ptr<Camera>( new CameraPinholeDistor(
      ((CameraPinhole*)((Camera*)cam_tmp))->getIntraMatrix(),
      radial_dist, nbRadialParam, tangential_dist) );
  }

  void CameraPinholeDistor::write( cv::FileStorage& fs ) const
  {
    fs << "CameraPinholeDistor" << "{";

    ((CameraPinhole*)this)->write( fs );

    fs << "radial_dist_" << "[:";
    for(int i=0;i<6;i++)
      fs << this->radial_dist_[i];
    fs << "]";

    fs << "nb_radial_params_" << this->nb_radial_params_;
    fs << "tangential_dist_" << "[:";
    for(int i=0;i<2;i++)
      fs << this->tangential_dist_[i];
    fs << "]";
    fs << "}";
  }
}
