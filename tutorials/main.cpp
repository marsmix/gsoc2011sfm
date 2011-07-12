

#include "test_data_sets.h"

int main()
{
  int choice = Tutorial_Handler::print_menu();
  while( choice>=0 )
  {
    Tutorial_Handler::run_tuto(choice);
    choice = Tutorial_Handler::print_menu();
  }
}
