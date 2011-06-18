#ifndef _GSOC_SFM_CAMERA_H
#define _GSOC_SFM_CAMERA_H 1


#include "opencv2/core/core.hpp"

namespace OpencvSfM{
  /**
  * \brief This class is a virtual base class for two view solvers, like fundamental matrix, essential matrix, triangulation...
  The only method needed is solve which take two matrix as input and one other as output.
  */
  class TwoViewSolver
  {
  protected:
    unsigned short minimum_inputs_;
  public:
    /**
    * Constructor of TwoViewSolver
    * @param minimum_inputs number of input needed to solve the problem
    * @return 
    */
    TwoViewSolver(unsigned short minimum_inputs=0);
    virtual ~TwoViewSolver(void);

    /**
    * used to know how many parameters we need to solve this problem:
    * @return number of minimal inputs
    */
    inline unsigned short getMinimInputs(){return minimum_inputs_;};

    /**
    * The function used to compute the solution.
    * @param 
    */
    virtual void solve()=0;
  };
}

#endif