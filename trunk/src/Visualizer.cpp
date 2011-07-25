
#include "Visualizer.h"

namespace OpencvSfM{
  Visualizer::Visualizer(std::string name)
    :viewer( new pcl::visualization::PCLVisualizer ( name ) )
  {

  }

}