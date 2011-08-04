
#include "test_data_sets.h"

using namespace OpencvSfM;
using namespace tutorials;
int main( )
{
  //usefull to hide libmv debug output...
  std::ofstream out("libmv_log.txt"); 
  std::clog.rdbuf(out.rdbuf()); 

  int choice = Tutorial_Handler::print_menu( );
  while( choice>=0 )
  {
    Tutorial_Handler::run_tuto( choice );
    choice = Tutorial_Handler::print_menu( );
  }
}
