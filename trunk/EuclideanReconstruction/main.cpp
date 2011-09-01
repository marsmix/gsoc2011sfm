
#include "macro.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/filesystem.hpp>   // includes all needed Boost.Filesystem declarations

#include "EuclideanEstimator.h"
#include "CameraPinholeDistor.h"
#include "PointOfView.h"


using namespace std;
using namespace OpencvSfM;
using namespace cv;
using namespace boost::filesystem;

string convertReal( string name )
{
  {
    path fileTmp( name.c_str( ) );
    if ( exists( fileTmp ) )
      return name;
  }

  {
    string nameTmp= FROM_SRC_ROOT(name);
    path fileTmp( nameTmp.c_str( ) );
    if ( exists( fileTmp ) )
      return nameTmp;
  }
  CV_Error(CV_StsError,(((string)"\nCan't open the file ")+name).c_str() );
  return "";
}

//Medias/modelHouse/house.000.P
vector<PointOfView> loadCamerasModelHouse( string nameFile )
{
  vector<PointOfView> cameras;
  string beforeName = nameFile.substr(0,nameFile.length()-3);
  std::ifstream inCams;
  int idFile = 0;
  stringstream nameStreamFile;
  nameStreamFile<<beforeName<<idFile<<".P\0";
  inCams.open( nameStreamFile.str().c_str( ) );
  double P[ 12 ];
  while( inCams.is_open() )
  {
    inCams >> P[ 0 ] >> P[ 1 ] >> P[ 2 ] >> P[ 3 ];
    inCams >> P[ 4 ] >> P[ 5 ] >> P[ 6 ] >> P[ 7 ];
    inCams >> P[ 8 ] >> P[ 9 ] >> P[10 ] >> P[11 ];
    //create the PointOfView:
    PointOfView myPointOfView( Mat( 3, 4, CV_64F, P ) );
    myPointOfView.setRotationMatrix( cv::Mat::eye(3,3,CV_64F) );
    myPointOfView.setTranslationVector( cv::Mat::zeros(3,1,CV_64F) );
    cameras.push_back( PointOfView ( Mat( 3, 4, CV_64F, P ) ) );

    idFile++;
    stringstream nameFile;
    nameFile<<beforeName<<idFile<<".P\0";
    string tmpName= nameFile.str();
    inCams.close();
    inCams.open( tmpName.c_str( ) );
  }
  return cameras;
}

//Medias/modelHouse/house.000.P
vector<PointOfView> loadCamerasTemple( string nameFile )
{
  vector<PointOfView> outVect;
  ifstream pointsDef( nameFile.c_str( ) );
  bool isOK=pointsDef.is_open( );
  if ( !isOK ){
    cout<<"Can't find "<<nameFile<<"!\nPlease download the temple"
      "dataset from http://vision.middlebury.edu/mview/data/"<<endl;
  }
  //first get the numbers of cameras:
  int nbCameras;
  if( pointsDef>>nbCameras )
  {
    string name_of_picture;
    Mat intra_params,rotation;
    Vec3d translation;
    intra_params.create( 3, 3, CV_64F );
    rotation.create( 3, 3, CV_64F );
    double* data_intra_param=( double* )intra_params.data;
    double* data_rotation=( double* )rotation.data;
    for ( int i=0;i<nbCameras;i++ )
    {
      //first the name of image:
      if( pointsDef>>name_of_picture )
      {
        //the 9 values of K:
        for( int j=0;j<9;j++ )
          pointsDef>>data_intra_param[ j ];
        //the 9 values of rotation:
        for( int j=0;j<9;j++ )
          pointsDef>>data_rotation[ j ];
        //the 3 values of translation:
        for( int j=0;j<3;j++ )
          pointsDef>>translation[ j ];
        //now create a point of view:
        outVect.push_back(
          PointOfView( new CameraPinhole( intra_params ) ));
      }
    }
  }
  return outVect;
}

vector<PointOfView> loadCameras( string nameFile )
{
  if( nameFile.find(".P")!=string::npos )
    return loadCamerasModelHouse( nameFile );
  if( nameFile.find("_par.txt")!=string::npos )
    return loadCamerasTemple( nameFile );
}

int main( )
{
  //usefull to hide libmv debug output...
  std::ofstream out("libmv_log.txt"); 
  std::clog.rdbuf(out.rdbuf());

  int type_of_input = 1, loadCamera = 1;
  Mat K = Mat::eye(3,3,CV_64F);
  double* data_intra_param=( double* )K.data;
  string imageDirectory =  "Medias/bottle/",//"Medias/templeSparseRing/",
    methodDetect = "PyramidORB", methodExtract = "ORB", methodMatch = "FlannBased";
  MotionProcessor mp;
  Ptr<Camera> my_device;
  vector<Mat> images;
  vector< Ptr<PointsToTrack> > vec_point_for_track;
  vector<PointOfView> myCameras;
  
  
  cout<<"Welcome to the Structure from motion Application!"<<endl;
  cout<<"This application will try to find camera position and geometry automatically!"<<endl;
  cout<<"We need the intra parameters..."<<endl;
  cout<<"If they don't change across the sequence, you can give them here, else I will try to load them!"<<endl;
  cout<<"So, (0) give the value of one camera, (1) load cameras intra values? ";
  cout<<"Default : 1"<<endl;
  cin>>loadCamera;
  if( loadCamera == 1 )
  {
    //templeSparseRing/templeSR_par.txt
    string nameCameraFile = "Medias/modelHouse/house.000.P";
    cout<<"Where can I find camera matrix (if multiple files, give the first one, I will try to find other alone!) ?"<<endl;
    cout<<"Default : "<<nameCameraFile<<endl;
    cin>>nameCameraFile;
    myCameras = loadCameras( convertReal(nameCameraFile) );
  }
  if( loadCamera != 1 )
  {
    cout<<"Please first give us the configuration of the camera (intra parameters):"<<endl;
    cout<<"First the focal_x_ (value usually between 1000 and 2500): ";
    cin>>data_intra_param[0];
    cout<<"Now the focal_y_ (can be set equal to focal_x_): ";
    cin>>data_intra_param[4];
    cout<<"The principal_point_x_ (can be set to image width/2): ";
    cin>>data_intra_param[2];
    cout<<"The principal_point_y_ (can be set to image height/2): ";
    cin>>data_intra_param[5];
    cout<<"And finally the skew (can be set to 0): ";
    cin>>data_intra_param[1];

    my_device = new CameraPinhole( K );
  }

  cout<<endl<<"Which point detector you want to use (SIFT, SURF, ORB, FAST, HARRIS, ...)?"<<endl;
  cout<<"For a list of available detector, see\n"
    "http://code.google.com/p/gsoc2011sfm/wiki/PointsToTrack_tut"<<endl;
  cout<<"Default : "<<methodDetect<<endl;
  cin>>methodDetect;

  cout<<endl<<"Which point extractor you want to use (SIFT, SURF, ORB, BRIEF)?"<<endl;
  cout<<"For a list of available detector, see\n"
    "http://code.google.com/p/gsoc2011sfm/wiki/PointsToTrack_tut"<<endl;
  cout<<"Default : "<<methodExtract<<endl;
  cin>>methodExtract;

  cout<<endl<<"Which point matcher you want to use (BruteForce, FlannBased, BruteForce-Hamming, ...)?"<<endl;
  cout<<"For details and compatibility between matcher and descritors, see\n"
    "http://code.google.com/p/gsoc2011sfm/wiki/Matching_tuto"<<endl;
  cout<<"Default : "<<methodMatch<<endl;
  cin>>methodMatch;

  cout<<endl<<"The sequence you have is:\n(0) a video\n(1) a directory of images\n(2) a webcam?"<<endl;
  cout<<"Default : 1"<<endl;
  cin>>type_of_input;
  switch(type_of_input)
  {
    case 0:
      cout<<"So please give me the path of the video:"<<endl;
      cin>>imageDirectory;
      mp.setInputSource( convertReal(imageDirectory), IS_VIDEO );
      break;
    case 2:{
      int idWebcam;
      cout<<"So please give me the id of webcam (usually 0):"<<endl;
      cin>>idWebcam;
      mp.setInputSource( idWebcam );
      cout<<"Use letter 'S' to take a snapshot and 'B' to begin reconstruction!"<<endl;
      break;
           }
    default://by default, it's a directory...
      cout<<"So please give me the path of the folder containing the images:"<<endl;
      cout<<"Default : "<<imageDirectory<<endl;
      cin>>imageDirectory;
      mp.setInputSource( convertReal(imageDirectory), IS_DIRECTORY );
  }

  //////////////////////////////////////////////////////////////////////////
  //everything is now configured, we will be able to begin:
  //////////////////////////////////////////////////////////////////////////

  cout<<"Load images and compute interest points:"<<endl;
  Mat currentImage=mp.getFrame( );
  bool cam_added = false;
  while ( !currentImage.empty( ) )
  {
    cam_added = false;
    if( type_of_input != 2 )
    {//load each images
      images.push_back( currentImage );
      cam_added = true;
    }
    else
    {
      imshow( "Webcam output",currentImage );
      int keyPressed = waitKey( 25 );
      if( keyPressed == 'S' ){
        images.push_back( currentImage );
        cam_added = true;
      }
      if( keyPressed == 'B' )//quit the loop...
        currentImage = Mat();
      else
        images.push_back( currentImage );
    }
    if( cam_added )
    {
      if( loadCamera != 1 )
        myCameras.push_back( PointOfView( my_device ) );
      Ptr<PointsToTrack> ptrPoints_tmp =
        Ptr<PointsToTrack>( new PointsToTrackWithImage ( images.size()-1,
        currentImage, methodDetect, methodExtract ));
      ptrPoints_tmp->computeKeypointsAndDesc( true );
      if(images.size()>200)//reduce memory usage:
        ptrPoints_tmp->free_descriptors();
      vec_point_for_track.push_back( ptrPoints_tmp );

      Mat tmpImg;
      ptrPoints_tmp->printPointsOnImage(currentImage, tmpImg );
      imshow( "Loaded Image",tmpImg );
      waitKey( 25 );
    }
    currentImage=mp.getFrame( );
  }
  cv::destroyAllWindows();

  cout<<"Sequence loaded, will begin the reconstruction:"<<endl;
  cout<<"Compute matches between each frames..."<<endl;
  cout<<"The complexity is O( n^2 ), so be patient..."<<endl;

  //create the matcher:
  Ptr<PointsMatcher> matcher = PointsMatcherOpticalFlow::create( "OpticalFlowPyrLK" );
  //Ptr<PointsMatcher> matcher = PointsMatcher::create( methodMatch );
  //and the sequence analyzer:
  SequenceAnalyzer motion_estim( vec_point_for_track, &images, matcher );

  motion_estim.computeMatches( );
  SequenceAnalyzer::keepOnlyCorrectMatches(motion_estim,4,0);

  cout<<"numbers of correct tracks:"<<
    motion_estim.getTracks( ).size( )<<endl;

  //now create the euclidean estimator:
  EuclideanEstimator pe( motion_estim, myCameras );
  pe.computeReconstruction( );

  //finally show reconstruction:
  pe.viewEstimation( false );
}