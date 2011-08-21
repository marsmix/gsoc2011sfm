
#include "config_SFM.h"
#include "../src/macro.h"
#include "../src/PCL_mapping.h"

#include<Eigen/StdVector>
#include <vector>

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Only to see how we switch between PCL types and OpenCV ones
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
//using namespace cv;//Risk of overlaps!
//using namespace pcl;
using namespace std;

NEW_TUTO( PCL_tuto, "Learn how you can map values between PCL and OpenCV ",
  "We will show how we can switch between PCL objects and OpenCV objects." )
{

  mapping::Point testPoint;
  cv::Vec3f& testCV = testPoint;
  pcl::PointXYZ& testPCL = testPoint;

  //both point share the same memory space:
  testPCL.x = 12.0;//change PCL point ( and OpenCV too )
  testCV[ 1 ] = (float)(testCV[ 0 ] / 2.0);//Change OpenCV point ( Result: 6.0 )
  testPCL.y = 30;//change PCL point and Opencv too ( 30 )

  cout<<"As you will see, these points share the same memory space:"<<endl;
  cout<<testPCL.y<<" == "<<testCV[ 1 ]<<" : "<< testPoint.data_[ 1 ]<<endl<<endl;

  //but you can't convert to a cv::Vec3d ( values need conversion )
  try{
    cv::Vec3d& testCVdouble = testPoint;
  }catch( cv::Exception& e )
  {
    cout<<"Conversion error: "<<e.code<<endl;
  }

  //But we can do something like that:
  cv::Vec3d testCVdouble;
  mapping::convert_<double>( &testPoint, &testCVdouble );
  cout<<"The double value is : "<<testCVdouble[ 1 ]<<endl<<endl;

  //////////////////////////////////////////////////////////////////////////
  //Now we want to do conversion without data copy:
  pcl::PointXYZ my_PCL_Point;
  my_PCL_Point.x = 12.5; my_PCL_Point.y = 2.5; my_PCL_Point.z = 120.5;
  mapping::Point convertor( my_PCL_Point );
  cv::Vec3f& cv_point = convertor;
  //We can now play with cv_point:
  cv_point[ 1 ] = 3.5; cv_point[ 0 ] *= 10;
  cout<<"The cv::Vec3f has the following values:"<<endl;
  cout<<cv_point[ 0 ]<<", "<<cv_point[ 1 ]<<", "<< cv_point[ 2 ]<<endl;
  cout<<"and the pcl::PointXYZ has the following values:"<<endl;
  cout<<my_PCL_Point.x<<", "<<my_PCL_Point.y<<", "<< my_PCL_Point.z<<endl;

  //be careful that direct conversion from cv::Vec3f to pcl::PointXYZ
  //is not possible ( sizeof( cv::Vec3f ) < sizeof( pcl::PointXYZ )):
  cv::Vec3f cv_point_test;
  cv_point_test[ 0 ] = 120.5;
  mapping::Point convertorBis( cv_point_test );
  try{
    pcl::PointXYZ& my_PCL_Point_error = convertorBis;
  }catch( cv::Exception& e )
  {
    cout<<"Conversion error..."<<endl;
    cout<<e.file<<" "<<e.func<<" "<<e.line<<endl;
  }

  //////////////////////////////////////////////////////////////////////////
  //Now what can we do with vectors?
  //First a conversion to the generic type:
  vector< cv::Vec4f > my_vector_OpenCV;//get data from somewhere;
  my_vector_OpenCV.push_back( cv::Vec4f( 1,2,3,4 ) );
  my_vector_OpenCV.push_back( cv::Vec4f( 5,6,7,8 ) );
  my_vector_OpenCV.push_back( cv::Vec4f( 9,0,1,2 ) );
  vector< mapping::Point, Eigen::aligned_allocator<mapping::Point> > my_generic_points;
  convertToMappedVector( my_vector_OpenCV, my_generic_points );

  //Now a conversion from the generic type:
  std::vector<pcl::PointXYZ,
    Eigen::aligned_allocator<pcl::PointXYZ> > my_vector_converted;
  convertFromMappedVector( my_generic_points, my_vector_converted );
  cout<<"Values of converted vector:"<<endl;
  cout<<my_vector_converted[ 0 ]<<"; "<<
    my_vector_converted[ 1 ]<<"; "<<
    my_vector_converted[ 2 ]<<endl;

  //////////////////////////////////////////////////////////////////////////
  //We can do a conversion without having to use a vector of Point:
  std::vector<pcl::PointXYZ,
    Eigen::aligned_allocator<pcl::PointXYZ> > my_vector_PCL_bis;
  mapping::convert_OpenCV_vector( my_vector_OpenCV, my_vector_PCL_bis );
  cout<<"Values of directly converted vector ( should be the same ):"<<endl;
  cout<<my_vector_PCL_bis[ 0 ]<<"; "<<
    my_vector_PCL_bis[ 1 ]<<"; "<<
    my_vector_PCL_bis[ 2 ]<<endl;

  vector< cv::Vec4f > my_vector_OpenCV_bis;
  mapping::convert_PCL_vector( my_vector_PCL_bis, my_vector_OpenCV_bis );
  cout<<"Values of the openCV vector ( only the second value ):"<<endl;
  cout<<my_vector_OpenCV_bis[ 1 ][ 0 ]<<"; "<<
    my_vector_OpenCV_bis[ 1 ][ 1 ]<<"; "<<
    my_vector_OpenCV_bis[ 1 ][ 2 ]<<"; "<<
    my_vector_OpenCV_bis[ 1 ][ 3 ]<<endl;

  //////////////////////////////////////////////////////////////////////////
  //We can also do a "direct conversion" from a PCL vector to a cv::Mat :
  cv::Mat my_list_of_points;
  mapping::convert_PCL_vector( my_vector_converted, my_list_of_points );
  cout<<"The opencv matrix equal to:"<<endl<<my_list_of_points<<endl;
  //change one PCL point:
  my_vector_converted[ 1 ].y = (float)250.54;
  cout<<"The opencv matrix is now equal to:"<<endl<<my_list_of_points<<endl;
}
