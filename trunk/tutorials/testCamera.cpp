
#include "config.h"
#include "../src/CameraPinholeDistor.h"

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Problems with DISTORTION: unable to find a fast and accurate way to
//transform pixels in camera normalized coordinates into image pixels.
//The only way I found was to use LM iteration to find the original point...
//////////////////////////////////////////////////////////////////////////
#include "test_data_sets.h"

using namespace cv;
using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
using namespace std;

NEW_TUTO( Cam_creation, "Learn what you can do with a pinhole distorded camera",
  "Using intra parameters, switch between camera / image coordinates." )
{
  Mat intra=Mat::eye( 3, 3, CV_64F );
  double* data = ( double* )intra.data;

  data[ 0 ]=1500.0;
  data[ 4 ]=1535.0;
  data[ 2 ]=320.0;
  data[ 5 ]=240.0;

  Vec6d radialDist( 0.2,0.4,0.3,0.1,0.1,0.01 );
  Vec2d tangDist( 0.01,0.1 );

  Ptr<Camera> camera;
  cout<<"Choose the camera model you want to use:\n0 )Pinhole\n1 )Pinhole with distortions"<<endl;
  int choice;
  cin>>choice;
  switch( choice )
  {
  case 1:
    cout<<"Create a pinhole camera with distortions..."<<endl;
    camera = Ptr<Camera>( new CameraPinholeDistor( intra,radialDist,6,tangDist ));
    break;
  default:
    cout<<"Create a pinhole camera..."<<endl;
    camera = Ptr<Camera>( new CameraPinhole( intra ));

  }

  vector<Vec2d> points;//image pixel coords.

  points.push_back( Vec2d( 10,10 ));
  points.push_back( Vec2d( 100,10 ));
  points.push_back( Vec2d( 20,100 ));
  points.push_back( Vec2d( 250,500 ));
  points.push_back( Vec2d( 130,10 ));
  points.push_back( Vec2d( 450,200 ));

  vector<Vec2d> pointsImgCoord;//image coords.
  pointsImgCoord=camera->pixelToNormImageCoordinates( points );
  for( int i=0;i<6;i++ )
  {
    cout<<pointsImgCoord[ i ][ 0 ]<<" "<<pointsImgCoord[ i ][ 1 ]<<endl;
  }

  vector<Vec2d> pointsImgPixel;//back to pixel coord.
  pointsImgPixel=camera->normImageToPixelCoordinates( pointsImgCoord );
  cout<<"From normalized Image coordinates to image Pixel coordinates:"<<endl;
  for( int i=0;i<6;i++ )
  {
    cout<<pointsImgPixel[ i ][ 0 ]<<" "<<pointsImgPixel[ i ][ 1 ];
    cout<<" ( was "<<points[ i ][ 0 ]<<" "<<points[ i ][ 1 ]<<" )"<<endl;
  }

}
