#ifndef _GSOC_SFM_POINTS_BUNDLE_RELATED_H
#define _GSOC_SFM_POINTS_BUNDLE_RELATED_H 1

#include <vector>
#include <sba.h>
#include "libmv/numeric/numeric.h"
#include "libmv/base/vector.h"

//A lot of these functions are extracted from Lourakis' SBA:
//http://www.ics.forth.gr/~lourakis/sba/

#include "macro.h" //SFM_EXPORTS

namespace OpencvSfM{
  class SFM_EXPORTS SequenceAnalyzer;
  class SFM_EXPORTS PointOfView;

  /**
  * This structure help lourakis bundle adjustment to find needed information.
  */
  struct bundle_datas
  {
    libmv::vector< libmv::Mat3 >& intraParams;///<list of intra parameters of each cameras
    libmv::vector< Eigen::Quaterniond >& rotations;///<list of rotations matrix of each cameras
    libmv::vector< libmv::Vec3 >& translations;///<list of translation vector of each cameras
    double* points3D;///<List of 3d points
    int cnp;///<number of parameters for ONE camera; e.g. 6 for Euclidean cameras
    int pnp;///<number of parameters for ONE 3D point; e.g. 3 for Euclidean points
    int mnp;///<number of parameters for ONE projected point; e.g. 2 for Euclidean points
    int ncon;///<number of points (starting from the 1st) whose parameters should not be modified.
    int mcon;///<number of cameras (starting from the 1st) whose parameters should not be modified.

    /**
    * Construct a bundle helper object.
    * @param i list of intra parameters of each cameras
    * @param r list of rotations matrix of each cameras
    * @param t list of translation vector of each cameras
    * @param c number of parameters for ONE camera
    * @param p number of parameters for ONE 3D point
    * @param mp number of parameters for ONE projected point
    * @param n number of points (starting from the 1st) whose parameters should not be modified.
    * @param m number of cameras (starting from the 1st) whose parameters should not be modified.
    */
    bundle_datas(libmv::vector< libmv::Mat3 >& i,
      libmv::vector< Eigen::Quaterniond >& r,
      libmv::vector< libmv::Vec3 >& t,
      int c, int p, int mp, int n, int m)
      :intraParams(i),rotations(r),translations(t),cnp(c), pnp(p),mnp(mp), mcon(m),ncon(n)
    {
      points3D = NULL;
    }
  };

  void SFM_EXPORTS full_bundle( SequenceAnalyzer &sequence,
    std::vector<PointOfView>& cameras);

  void calcImgProjFullR(double a[5],double qr0[4],double t[3],double M[3],
    double n[2]);


  inline static void quatMultFast(double q1[4], double q2[4], double p[4])
  {
    double t1, t2, t3, t4, t5, t6, t7, t8, t9;
    //double t10, t11, t12;

    t1=(q1[0]+q1[1])*(q2[0]+q2[1]);
    t2=(q1[3]-q1[2])*(q2[2]-q2[3]);
    t3=(q1[1]-q1[0])*(q2[2]+q2[3]);
    t4=(q1[2]+q1[3])*(q2[1]-q2[0]);
    t5=(q1[1]+q1[3])*(q2[1]+q2[2]);
    t6=(q1[1]-q1[3])*(q2[1]-q2[2]);
    t7=(q1[0]+q1[2])*(q2[0]-q2[3]);
    t8=(q1[0]-q1[2])*(q2[0]+q2[3]);


    t9=0.5*(t5-t6+t7+t8);
    p[0]= t2 + t9-t5;
    p[1]= t1 - t9-t6;
    p[2]=-t3 + t9-t8;
    p[3]=-t4 + t9-t7;
  }

  void img_projsRTS_x(/*cameras and points*/ double *p,
  /*sparse matrix of 2D points*/ struct sba_crsm *idxij,
    /*tmp vector*/int *rcidxs, /*tmp vector*/int *rcsubs,
    /*out vect*/double *hx,
    void *adata);

  void img_projsKRTS_x(/*cameras and points*/ double *p,
  /*sparse matrix of 2D points*/ struct sba_crsm *idxij,
    /*tmp vector*/int *rcidxs, /*tmp vector*/int *rcsubs,
    /*out vect*/double *hx,
    void *adata);

  void img_projsRT_x(/*cameras and points*/ double *p,
  /*sparse matrix of 2D points*/ struct sba_crsm *idxij,
    /*tmp vector*/int *rcidxs, /*tmp vector*/int *rcsubs,
    /*out vect*/double *hx,
    void *adata);
  void calcImgProjJacRTS(double a[5],double qr0[4],double v[3],double t[3],
    double M[3],double jacmRT[2][6],double jacmS[2][3]);

  void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata);
}

#endif