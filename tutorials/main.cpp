
#include "test_data_sets.h"
#include "../src/PCL_mapping.h"

using namespace OpencvSfM;
using namespace cv;
using namespace pcl;
int main()
{
  Point3D_mapping testPoint;
  cv::Vec3f& testCV = testPoint;
  pcl::PointXYZ& testPCL = testPoint;

  //both point share the same memory space:
  testPCL.x = 12.0;//change PCL point (and OpenCV too)
  testCV[1] = testCV[0] / 2.0;//Change OpenCV point (Result: 6.0)
  testPCL.y = 30;//change PCL point and Opencv too

  cout<<testPCL.y<<" -> "<<testCV[1]<<" ; "<< testPoint.data_[1]<<endl;

  //but we can also create distinct memory space:
  cv::Vec3f testCV1 = testPoint;
  pcl::PointXYZ testPCL1 = testPoint;

  //points have distinct memory space:
  testPCL1.x = 24.0;//change PCL point (and not OpenCV)
  testCV1[1] = testCV1[0] / 2.0;//Change OpenCV point
  testPCL1.y = 20;//change PCL point

  cout<<testPCL1.y<<" -> "<<testCV1[1]<<" ; "<< testPoint.data_[1]<<endl;

  int choice = Tutorial_Handler::print_menu();
  while( choice>=0 )
  {
    Tutorial_Handler::run_tuto(choice);
    choice = Tutorial_Handler::print_menu();
  }
}