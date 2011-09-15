#if ( defined _DEBUG && defined _MSC_VER)
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif

#include <iostream>

#include "test_data_sets.h"

using namespace std;
using namespace OpencvSfM;
using namespace tutorials;
int main( )
{
#if ( defined _DEBUG && defined _MSC_VER)
  _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
  cout<<"look for memory leak..."<<endl;
  //_CrtSetBreakAlloc(1134);
#endif
  
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