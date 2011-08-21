

#include <pcl/point_types.h>
#include "libmv/multiview/five_point.h"
#include "libmv/multiview/affine.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/robust_fundamental.h"
#include "libmv/multiview/robust_euclidean_resection.h"
#include "libmv/multiview/bundle.h"
#include <sba.h>

#include <Eigen/Eigenvalues>

#include <pcl/io/vtk_io.h>
#include <sstream>

#include "EuclideanEstimator.h"
#include "StructureEstimator.h"
#include "Camera.h"
#include "Visualizer.h"
#include "PCL_mapping.h"

using std::vector;
using cv::Ptr;

namespace OpencvSfM{
  //the next two functions are only for intern usage, no external interface...

  // Approximation of reprojection error;
  // The one from libmv don't use Mat2X as input
  double SampsonDistance2( const libmv::Mat &F,
    const libmv::Mat2X &x1, const libmv::Mat2X &x2 ) {
    double error_total= 0.0;
    libmv::Vec2 x1_i,x2_i;
    unsigned int n_points = x1.rows( );
    for( unsigned int i = 0; i < n_points ; ++i )
    {
      x1_i = x1.col( i );
      x2_i = x2.col( i );

      error_total += libmv::SampsonDistance( F, x1_i, x2_i );
    }

    return error_total;
  }

  //Idea from Snavely : Modeling the World from Internet Photo Collections
  //See with libmv team if such a function is usefull:
  double robust5Points( const libmv::Mat2X &x1, const libmv::Mat2X &x2,
    const libmv::Mat3 &K1, const libmv::Mat3 &K2,
    libmv::Mat3 &E )
  {
    unsigned int nPoints = x1.cols( );
    CV_DbgAssert( nPoints == x2.cols( ) );
    CV_DbgAssert( nPoints >= 5 );//need 5 points!

    cv::RNG& rng = cv::theRNG( );
    vector<int> masks( nPoints );
    double max_error = 1e9;

    int num_iter=0, max_iter=1500;
    for( num_iter=0; num_iter<max_iter; ++num_iter )
    {
      masks.assign( nPoints, 0 );
      int nb_vals=0;
      //choose 5 random points:
      while( nb_vals < 5 )
      {
        int valTmp = rng( nPoints );
        if( masks[ valTmp ] == 0 )
        {
          masks[ valTmp ] = 1;
          nb_vals++;
        }
      }
      //mix the random generator:
      rng( rng( nPoints ) );
      //create mask:
      libmv::Mat2X x1_tmp,x2_tmp;
      x1_tmp.resize( 2,nb_vals );
      x2_tmp.resize( 2,nb_vals );
      nb_vals=0;
      unsigned int i;
      for( i = 0; i<nPoints; ++i )
      {
        if( masks[ i ] != 0 )
        {
          x1_tmp( 0,nb_vals ) = x1( 0,i );
          x1_tmp( 1,nb_vals ) = x1( 1,i );
          x2_tmp( 0,nb_vals ) = x2( 0,i );
          x2_tmp( 1,nb_vals ) = x2( 1,i );
          nb_vals++;
        }
      }
      libmv::vector<libmv::Mat3, Eigen::aligned_allocator<libmv::Mat3> > Es(10);
      libmv::FivePointsRelativePose( x1_tmp,x2_tmp,&Es );
      unsigned int num_hyp = Es.size( );
      for ( i = 0; i < num_hyp; i++ ) {

        libmv::Mat3 F;
        libmv::FundamentalFromEssential( Es[ i ], K1, K2, &F );
        double error = SampsonDistance2( F, x1, x2 );

        if ( max_error > error ) {
          max_error = error;
          E = Es[ i ];
        }
      }
    }
    return max_error;
  }


  EuclideanEstimator::EuclideanEstimator( SequenceAnalyzer &sequence,
    vector<PointOfView>& cameras )
    :sequence_( sequence ),cameras_( cameras )
  {
    vector<PointOfView>::iterator itPoV=cameras.begin( );
    while ( itPoV!=cameras.end( ) )
    {
      addNewPointOfView( *itPoV );
      itPoV++;
    }
    index_origin = 0;
  }

  EuclideanEstimator::~EuclideanEstimator( void )
  {
    //TODO!!!!
  }

  void EuclideanEstimator::addNewPointOfView( const PointOfView& camera )
  {
    libmv::Mat3 intra_param;
    cv::Ptr<Camera> intra=camera.getIntraParameters( );
    //transpose because libmv needs intra params this way...
    cv::cv2eigen( intra->getIntraMatrix( ).t(), intra_param );
    intra_params_.push_back( intra_param );
    libmv::Mat3 rotation_mat;
    cv::cv2eigen( camera.getRotationMatrix( ), rotation_mat );
    rotations_.push_back( rotation_mat );
    libmv::Vec3 translation_vec;
    cv::cv2eigen( camera.getTranslationVector( ), translation_vec );
    translations_.push_back( translation_vec );
    camera_computed_.push_back( false );
  }

  struct bundle_datas
  {
    libmv::vector< libmv::Mat3 >& intraParams;
    libmv::vector< Eigen::Quaterniond >& rotations;
    libmv::vector< libmv::Vec3 >& translations;
    double* initial_projections;
    double* points3D;
    double* projections;
    int cnp, pnp, mnp, ncon, mcon;

    bundle_datas(libmv::vector< libmv::Mat3 >& i,
      libmv::vector< Eigen::Quaterniond >& r,
      libmv::vector< libmv::Vec3 >& t,
      int c, int p, int mp, int n, int m)
      :intraParams(i),rotations(r),translations(t),cnp(c), pnp(p),mnp(mp), mcon(m),ncon(n)
    {
      points3D = NULL;
      projections = NULL;
      initial_projections = NULL;
    }
  };


  void calcImgProjFullR(double a[5],double qr0[4],double t[3],double M[3],
    double n[2])
  {
    double t1;
    double t11;
    double t13;
    double t17;
    double t2;
    double t22;
    double t27;
    double t3;
    double t38;
    double t46;
    double t49;
    double t5;
    double t6;
    double t8;
    double t9;
    {
      t1 = a[0];
      t2 = qr0[1];
      t3 = M[0];
      t5 = qr0[2];
      t6 = M[1];
      t8 = qr0[3];
      t9 = M[2];
      t11 = -t3*t2-t5*t6-t8*t9;
      t13 = qr0[0];
      t17 = t13*t3+t5*t9-t8*t6;
      t22 = t6*t13+t8*t3-t9*t2;
      t27 = t13*t9+t6*t2-t5*t3;
      t38 = -t5*t11+t13*t22-t27*t2+t8*t17+t[1];
      t46 = -t11*t8+t13*t27-t5*t17+t2*t22+t[2];
      t49 = 1/t46;
      n[0] = (t1*(-t2*t11+t13*t17-t22*t8+t5*t27+t[0])+a[4]*t38+a[1]*t46)*t49;
      n[1] = (t1*a[3]*t38+a[2]*t46)*t49;
      return;
    }
  }


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
    void *adata)
  {
    //as we do an euclidean bundle adjustement, the adata contain constant params.
    bundle_datas* datas = ((bundle_datas*)adata);
    register int i, j;
    int cnp = datas->cnp, pnp = datas->pnp, mnp = datas->mnp;
    double *pa, *pb, *pqr, *pt, *ppt, *pmeas, lrot[4], trot[4];
    //int n;
    int m, nnz;

    m=idxij->nc;
    pa=p; pb=p+m*cnp;

    for(j=0; j<m; ++j){
      /* j-th camera parameters */
      libmv::Mat3 rotation;
      libmv::Mat3& K = datas->intraParams[j];
      Eigen::Quaterniond rot_init = datas->rotations[j];

      double Kparms[] = {K( 0,0 ),K( 2,0 ),K( 2,1 ),K( 1,1 )/K( 0,0 ),K( 1,0 )};
      // full quat for initial rotation estimate:
      double pr0[] = {rot_init.w(), rot_init.x(), rot_init.y(), rot_init.z()};
      libmv::Mat34 proj;

      pqr=pa+j*cnp;
      pt=pqr+3; // quaternion vector part has 3 elements
      libmv::Vec3 translat = datas->translations[j];
      double trans[3] = {
        pt[0] + translat(0),
        pt[1] + translat(1),
        pt[2] + translat(2)};
        double dist = sqrt( pt[0]*pt[0] + pt[1]*pt[1] + pt[2]*pt[2] );
      lrot[1]=pqr[0]; lrot[2]=pqr[1]; lrot[3]=pqr[2];
      lrot[0]=(1.0 - lrot[1]*lrot[1] - lrot[2]*lrot[2]- lrot[3]*lrot[3]);
      if( lrot[0]>0 )
        lrot[0] = sqrt( lrot[0] );
      else{//problem with this rotation...
        lrot[0] = 0;
        Eigen::Quaterniond quat_delta( lrot[0], lrot[1], lrot[2], lrot[3] );
        quat_delta.normalize();
        lrot[1]=quat_delta.x(); lrot[2]=quat_delta.y(); lrot[3]=quat_delta.z();
        lrot[0] = quat_delta.w();
      }

      quatMultFast(lrot, pr0, trot); // trot=lrot*pr0

      nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */

      for(i=0; i<nnz; ++i){
        ppt=pb + rcsubs[i]*pnp;
        pmeas=hx + idxij->val[rcidxs[i]]*mnp; // set pmeas to point to hx_ij

        calcImgProjFullR(Kparms, trot, trans, ppt, pmeas); // evaluate Q in pmeas
        pmeas[0] += dist/10;
        pmeas[1] += dist/10;
      }
    }
  }
  void img_projsRT_x(/*cameras and points*/ double *p,
  /*sparse matrix of 2D points*/ struct sba_crsm *idxij,
    /*tmp vector*/int *rcidxs, /*tmp vector*/int *rcsubs,
    /*out vect*/double *hx,
    void *adata)
  {
    //as we do an euclidean bundle adjustement, the adata contain constant params.
    bundle_datas* datas = ((bundle_datas*)adata);
    register int i, j;
    int cnp = datas->cnp, pnp = datas->pnp, mnp = datas->mnp;
    double *pa, *pb, *pqr, *pt, *ppt, *pmeas, lrot[4], trot[4];
    //int n;
    int m, nnz;

    m=idxij->nc;
    pa=p;

    pb = datas->points3D;

    for(j=0; j<m; ++j){
      /* j-th camera parameters */
      libmv::Mat3 rotation;
      libmv::Mat3& K = datas->intraParams[j];
      Eigen::Quaterniond rot_init = datas->rotations[j];

      double Kparms[] = {K( 0,0 ),K( 2,0 ),K( 2,1 ),K( 1,1 )/K( 0,0 ),K( 1,0 )};
      // full quat for initial rotation estimate:
      double pr0[] = {rot_init.w(), rot_init.x(), rot_init.y(), rot_init.z()};
      libmv::Mat34 proj;

      pqr=pa+j*cnp;
      pt=pqr+3; // quaternion vector part has 3 elements 
      libmv::Vec3 translat = datas->translations[j];
      double trans[3] = {
        pt[0] + translat(0),
        pt[1] + translat(1),
        pt[2] + translat(2)};
      lrot[1]=pqr[0]; lrot[2]=pqr[1]; lrot[3]=pqr[2];
      lrot[0]=(1.0 - lrot[1]*lrot[1] - lrot[2]*lrot[2]- lrot[3]*lrot[3]);
      if( lrot[0]>0 )
        lrot[0] = sqrt( lrot[0] );
      else{//problem with this rotation...
        lrot[0] = 0;
        Eigen::Quaterniond quat_delta( lrot[0], lrot[1], lrot[2], lrot[3] );
        quat_delta.normalize();
        lrot[1]=quat_delta.x(); lrot[2]=quat_delta.y(); lrot[3]=quat_delta.z();
        lrot[0] = quat_delta.w();
      }

      quatMultFast(lrot, pr0, trot); // trot=lrot*pr0

      nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */

      for(i=0; i<nnz; ++i){
        ppt=pb + rcsubs[i]*pnp;
        pmeas=hx + idxij->val[rcidxs[i]]*mnp; // set pmeas to point to hx_ij
        double* x = datas->initial_projections + idxij->val[rcidxs[i]]*mnp;
        calcImgProjFullR(Kparms, trot, trans, ppt, pmeas); // evaluate Q in pmeas
      }
    }
  }
  void calcImgProjJacRTS(double a[5],double qr0[4],double v[3],double t[3],
    double M[3],double jacmRT[2][6],double jacmS[2][3])
  {
    double t1;
    double t10;
    double t107;
    double t109;
    double t11;
    double t118;
    double t12;
    double t126;
    double t127;
    double t14;
    double t141;
    double t145;
    double t146;
    double t147;
    double t15;
    double t150;
    double t152;
    double t159;
    double t16;
    double t162;
    double t165;
    double t168;
    double t170;
    double t172;
    double t175;
    double t18;
    double t180;
    double t185;
    double t187;
    double t19;
    double t192;
    double t194;
    double t2;
    double t206;
    double t21;
    double t216;
    double t22;
    double t227;
    double t23;
    double t230;
    double t233;
    double t235;
    double t237;
    double t240;
    double t245;
    double t25;
    double t250;
    double t252;
    double t257;
    double t259;
    double t27;
    double t271;
    double t28;
    double t281;
    double t293;
    double t294;
    double t296;
    double t299;
    double t3;
    double t30;
    double t302;
    double t303;
    double t305;
    double t306;
    double t309;
    double t324;
    double t325;
    double t327;
    double t330;
    double t331;
    double t347;
    double t35;
    double t350;
    double t37;
    double t4;
    double t43;
    double t49;
    double t5;
    double t51;
    double t52;
    double t54;
    double t56;
    double t6;
    double t61;
    double t65;
    double t7;
    double t70;
    double t75;
    double t76;
    double t81;
    double t82;
    double t87;
    double t88;
    double t9;
    double t93;
    double t94;
    double t98;
    {
      t1 = a[0];
      t2 = v[0];
      t3 = t2*t2;
      t4 = v[1];
      t5 = t4*t4;
      t6 = v[2];
      t7 = t6*t6;
      t9 = sqrt(1.0-t3-t5-t7);
      t10 = 1/t9;
      t11 = qr0[1];
      t12 = t11*t10;
      t14 = qr0[0];
      t15 = -t12*t2+t14;
      t16 = M[0];
      t18 = qr0[2];
      t19 = t18*t10;
      t21 = qr0[3];
      t22 = -t19*t2-t21;
      t23 = M[1];
      t25 = t10*t21;
      t27 = -t25*t2+t18;
      t28 = M[2];
      t30 = -t15*t16-t22*t23-t27*t28;
      t35 = -t9*t11-t2*t14-t4*t21+t6*t18;
      t37 = -t35;
      t43 = t9*t18+t4*t14+t6*t11-t2*t21;
      t49 = t9*t21+t6*t14+t2*t18-t11*t4;
      t51 = -t37*t16-t43*t23-t49*t28;
      t52 = -t15;
      t54 = t10*t14;
      t56 = -t54*t2-t11;
      t61 = t9*t14-t2*t11-t4*t18-t6*t21;
      t65 = t61*t16+t43*t28-t23*t49;
      t70 = t56*t16+t22*t28-t23*t27;
      t75 = t56*t23+t27*t16-t28*t15;
      t76 = -t49;
      t81 = t61*t23+t49*t16-t37*t28;
      t82 = -t27;
      t87 = t56*t28+t23*t15-t22*t16;
      t88 = -t43;
      t93 = t61*t28+t37*t23-t43*t16;
      t94 = -t22;
      t98 = a[4];
      t107 = t30*t88+t94*t51+t56*t81+t61*t75+t87*t35+t93*t52-t70*t76-t82*t65;
      t109 = a[1];
      t118 = t30*t76+t82*t51+t56*t93+t61*t87+t70*t88+t65*t94-t35*t75-t81*t52;
      t126 = t76*t51+t61*t93+t65*t88-t81*t35+t[2];
      t127 = 1/t126;
      t141 = t51*t88+t61*t81+t93*t35-t65*t76+t[1];
      t145 = t126*t126;
      t146 = 1/t145;
      t147 = (t1*(t35*t51+t61*t65+t81*t76-t93*t88+t[0])+t98*t141+t126*t109)*t146;
      jacmRT[0][0] = (t1*(t30*t35+t52*t51+t56*t65+t61*t70+t76*t75+t81*t82-t88*t87
        -t93*t94)+t98*t107+t109*t118)*t127-t118*t147;
      t150 = t1*a[3];
      t152 = a[2];
      t159 = (t150*t141+t126*t152)*t146;
      jacmRT[1][0] = (t107*t150+t152*t118)*t127-t159*t118;
      t162 = -t12*t4+t21;
      t165 = -t19*t4+t14;
      t168 = -t25*t4-t11;
      t170 = -t162*t16-t165*t23-t168*t28;
      t172 = -t162;
      t175 = -t54*t4-t18;
      t180 = t175*t16+t165*t28-t168*t23;
      t185 = t175*t23+t168*t16-t162*t28;
      t187 = -t168;
      t192 = t175*t28+t162*t23-t165*t16;
      t194 = -t165;
      t206 = t170*t88+t51*t194+t175*t81+t61*t185+t192*t35+t93*t172-t76*t180-t65*
        t187;
      t216 = t170*t76+t51*t187+t93*t175+t61*t192+t180*t88+t65*t194-t185*t35-t81*
        t172;
      jacmRT[0][1] = (t1*(t170*t35+t172*t51+t175*t65+t180*t61+t185*t76+t81*t187-
        t192*t88-t93*t194)+t98*t206+t109*t216)*t127-t147*t216;
      jacmRT[1][1] = (t150*t206+t152*t216)*t127-t159*t216;
      t227 = -t12*t6-t18;
      t230 = -t19*t6+t11;
      t233 = -t25*t6+t14;
      t235 = -t227*t16-t23*t230-t233*t28;
      t237 = -t227;
      t240 = -t54*t6-t21;
      t245 = t240*t16+t230*t28-t233*t23;
      t250 = t23*t240+t233*t16-t227*t28;
      t252 = -t233;
      t257 = t240*t28+t227*t23-t230*t16;
      t259 = -t230;
      t271 = t235*t88+t51*t259+t81*t240+t61*t250+t257*t35+t93*t237-t245*t76-t65*
        t252;
      t281 = t235*t76+t51*t252+t240*t93+t61*t257+t245*t88+t259*t65-t250*t35-t81*
        t237;
      jacmRT[0][2] = (t1*(t235*t35+t237*t51+t240*t65+t61*t245+t250*t76+t81*t252-
        t257*t88-t93*t259)+t271*t98+t281*t109)*t127-t147*t281;
      jacmRT[1][2] = (t150*t271+t281*t152)*t127-t159*t281;
      jacmRT[0][3] = t127*t1;
      jacmRT[1][3] = 0.0;
      jacmRT[0][4] = t98*t127;
      jacmRT[1][4] = t150*t127;
      jacmRT[0][5] = t109*t127-t147;
      jacmRT[1][5] = t152*t127-t159;
      t293 = t35*t35;
      t294 = t61*t61;
      t296 = t88*t88;
      t299 = t35*t88;
      t302 = t61*t76;
      t303 = 2.0*t299+t61*t49-t302;
      t305 = t35*t76;
      t306 = t61*t88;
      t309 = t305+2.0*t306-t49*t35;
      jacmS[0][0] = (t1*(t293+t294+t49*t76-t296)+t98*t303+t109*t309)*t127-t147*
        t309;
      jacmS[1][0] = (t150*t303+t152*t309)*t127-t159*t309;
      t324 = t76*t76;
      t325 = t296+t294+t35*t37-t324;
      t327 = t76*t88;
      t330 = t61*t35;
      t331 = 2.0*t327+t61*t37-t330;
      jacmS[0][1] = (t1*(t299+2.0*t302-t37*t88)+t98*t325+t109*t331)*t127-t147*
        t331;
      jacmS[1][1] = (t150*t325+t152*t331)*t127-t159*t331;
      t347 = t327+2.0*t330-t43*t76;
      t350 = t324+t294+t43*t88-t293;
      jacmS[0][2] = (t1*(2.0*t305+t61*t43-t306)+t98*t347+t350*t109)*t127-t147*
        t350;
      jacmS[1][2] = (t150*t347+t152*t350)*t127-t159*t350;
      return;
    }
  }

  static void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata)
  {
    bundle_datas* datas = ((bundle_datas*)adata);
    register int i, j;
    int cnp = datas->cnp, pnp = datas->pnp, mnp = datas->mnp;

    double *pa, *pb, *pqr, *pt, *ppt, *pA, *pB;
    //int n;
    int m, nnz, Asz, Bsz, ABsz;

    //n=idxij->nr;
    m=idxij->nc;
    pa=p; pb=p+m*cnp;
    Asz=mnp*cnp; Bsz=mnp*pnp; ABsz=Asz+Bsz;

    for(j=0; j<m; ++j){
      /* j-th camera parameters */
      pqr=pa+j*cnp;
      pt=pqr+3; // quaternion vector part has 3 elements
      libmv::Vec3 translat = datas->translations[j];
      double vec_translat[3] = {
        pt[0] + translat(0),
        pt[1] + translat(1),
        pt[2] + translat(2)};
      libmv::Mat3 rotation;
      libmv::Mat3& K = datas->intraParams[j];
      Eigen::Quaterniond rot_init = datas->rotations[j];

      double Kparms[] = {K( 0,0 ),K( 2,0 ),K( 2,1 ),K( 1,1 )/K( 0,0 ),K( 1,0 )};
      // full quat for initial rotation estimate:
      double pr0[] = {rot_init.w(), rot_init.x(), rot_init.y(), rot_init.z()};
      libmv::Mat34 proj;

      nnz=sba_crsm_col_elmidxs(idxij, j, rcidxs, rcsubs); /* find nonzero hx_ij, i=0...n-1 */

      for(i=0; i<nnz; ++i){
        ppt=pb + rcsubs[i]*pnp;
        pA=jac + idxij->val[rcidxs[i]]*ABsz; // set pA to point to A_ij
        pB=pA  + Asz; // set pB to point to B_ij

        calcImgProjJacRTS(Kparms, pr0, pqr, vec_translat, ppt, (double (*)[6])pA, (double (*)[3])pB); // evaluate dQ/da, dQ/db in pA, pB
      }
    }
  }


#define VISIB_MASK i+j*m
#define VISIB_MASK_new i+j_real*m

  void EuclideanEstimator::bundleAdjustement( )
  {
    //wrap the lourakis SBA:
    
    unsigned int n = point_computed_.size( ),   // number of points
      ncon = 0,// number of points (starting from the 1st) whose parameters should not be modified.
      m = 0,   // number of images (or camera)
      mcon = 1,// number of cameras (starting from the 1st) whose parameters should not be modified.
      cnp = 6,// number of parameters for ONE camera; e.g. 6 for Euclidean cameras
      //use only vector part of quaternion to enforce the unit lenght...
      pnp = 3,// number of parameters for ONE 3D point; e.g. 3 for Euclidean points
      mnp = 2;// number of parameters for ONE projected point; e.g. 2 for Euclidean points

    unsigned int i = 0, j = 0,
      nb_cam = camera_computed_.size( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );

    //because some points are sometime not visible:
    vector<int> idx_cameras;
    idx_cameras.push_back( index_origin );

    libmv::vector< libmv::Mat3 > intra_p;
    std::vector<bool> pointOK;
    int nbPoints = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      //test if at least 2 views see this point:
      int nbCam = 0;
      for(size_t k =0; k<nb_cam; ++k)
      {
        if(camera_computed_[ k ] && point_computed_[ j ].containImage( k ))
          nbCam++;
      }
      pointOK.push_back( nbCam>=2 );
      if(pointOK[j])
        nbPoints++;
    }

    int nz_count = 0;
    for ( i = 0; i < nb_cam; ++i )
    {//for each camera:
      if( camera_computed_[ i ] )
      {
        if( i!=index_origin )//index_origin is already added...
          idx_cameras.push_back(i);
        m++;//increament of camera count

        int nb_projection = 0;
        for ( j = 0; j < n; ++j )
        {//for each 3D point:
          if( pointOK[j])
          {
            if( point_computed_[ j ].containImage( i ) )
              nb_projection++;
          }
        }
        nz_count += nb_projection;
      }
    }
    n=nbPoints;
    
    //2D points:
    char *vmask = new char[ n*m ];//visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise.
    double *p = new double[m*cnp + n*pnp];//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
                   // aj are the image j parameters, bi are the i-th point parameters

    double *x = new double[ 2*nz_count ];// measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
                   // x_ij is the projection of the i-th point on the j-th image.
                   // NOTE: some of the x_ij might be missing, if point i is not visible in image j;
                   // see vmask[i, j], max. size n*m*mnp

    libmv::vector< Eigen::Quaterniond > init_rotation;
    libmv::vector< libmv::Vec3 > init_translat;
    //update each variable:
    int idx_visible = 0;
    double *p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      intra_p.push_back( intra_params_[idx_cam] );
      //extrinsic parameters only (intra are know in euclidean reconstruction)
      init_rotation.push_back( (Eigen::Quaterniond)rotations_[ idx_cam ] );
      init_translat.push_back( translations_[ idx_cam ] );
      //add camera parameters to p:
      //as this is rotation, the quaternion's length is unity. Only 3 values are needed.
      //4th value equal:
      //sqrt(1.0 - quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2]));

      p_local[0] = 0; p_local[1] = 0; p_local[2] = 0;

      p_local[3] = 0; p_local[4] = 0; p_local[5] = 0;

      p_local+=cnp;
    }

    //now add the projections and 3D points:
    idx_visible = 0;
    int j_real = 0;
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        for ( i=0; i < m; ++i )
        {//for each camera:
          int idx_cam = idx_cameras[i];
          vmask[ VISIB_MASK_new ] = point_computed_[ j ].containImage( idx_cam );
          if( vmask[ VISIB_MASK_new ] )
          {
            cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
              point_computed_[ j ].getPointIndex( idx_cam ) );
            x[ idx_visible++ ] = pt.pt.x;
            x[ idx_visible++ ] = pt.pt.y;
          }
        }
        j_real++;
      }
    }
    double* points3D_values = p_local;
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        cv::Ptr<cv::Vec3d> cv3DPoint = point_computed_[ j ].get3DPosition();
        if(cv3DPoint.empty())
        {
          *(p_local++) = 0;
          *(p_local++) = 0;
          *(p_local++) = 0;
        }
        else
        {
          *(p_local++) = (*cv3DPoint)[ 0 ];
          *(p_local++) = (*cv3DPoint)[ 1 ];
          *(p_local++) = (*cv3DPoint)[ 2 ];
        }
      }
    }

    bundle_datas data(intra_p,init_rotation,init_translat,
      cnp, pnp, mnp,ncon, mcon);
    data.points3D = points3D_values;
    data.projections = x;

    //////////////////////////////////////////////////////////////////////////
#define PRINT_DEBUG
#ifdef PRINT_DEBUG
    //Debug compare projected point vs estimated point:
    idx_visible = 0;
    double max_distance = 0;
    double max_depth = 0;
    j_real = 0;
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        for ( i=0; i < m; ++i )
        {//for each camera:
          //2D projected points
          if( vmask[ VISIB_MASK_new ] )
          {
            cv::Vec3d& cv3DPoint = point_computed_[ j ];
            //cout<<"Vec3d : "<< cv3DPoint[ 0 ]<<", "<< cv3DPoint[ 1 ]<<", "<< cv3DPoint[ 2 ]<<endl;
            int idx_cam = idx_cameras[i];
            cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
              point_computed_[ j ].getPointIndex( idx_cam ) );
            cv::Vec2d proj = cameras_[ idx_cam ].project3DPointIntoImage(cv3DPoint);/*
            cout<<pt.pt.x<<","<<pt.pt.y<<" -> ";
            cout<<proj[0]<<","<<proj[1]<<endl;*/
            max_distance += (pt.pt.x - proj[0])*(pt.pt.x - proj[0]) +
              (pt.pt.y - proj[1])*(pt.pt.y - proj[1]);

            libmv::Vec3 X(cv3DPoint[0],cv3DPoint[1],cv3DPoint[2]);
            max_depth += abs( (rotations_[ idx_cam ]*X)(2) + translations_[ idx_cam ](2) );
          }
        }
        j_real++;
      }
      //system("pause");
    }
#endif
    ////////////////////////////////////////////////////////////////////////*/

    //TUNING PARAMETERS:
    int itmax = 10000;        //max iterations
    int verbose = 1;
    double opts[SBA_OPTSSZ] = {
      0.001,		//Tau
      1e-20,		//E1
      1e-20,		//E2
      0,		//E3 average reprojection error
      0		//E4 relative reduction in the RMS reprojection error
    };

    double info[SBA_INFOSZ];
    
    //use sba library
    int iter = sba_motstr_levmar_x(n, ncon, m, mcon, vmask, p, cnp, pnp, x, NULL, mnp,
        img_projsRTS_x, img_projsRTS_jac_x, (void*)&data, itmax, 0, opts, info);

    std::cout<<"SBA returned in "<<iter<<" iter, reason "<<info[6]
    <<", error "<<info[1]<<" [initial "<< info[0]<<"]\n";
    if(iter>1)
    {
    //set new values:
    m = idx_cameras.size();
    n = point_computed_.size( );
    idx_visible = 0;
    p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      //extrinsic parameters only (intra are know in euclidean reconstruction)

      Eigen::Quaterniond rot_init = data.rotations[i];
      double c1 = p_local[0];
      double c2 = p_local[1];
      double c3 = p_local[2];
      double coef=(1.0 - c1*c1 - c2*c2 - c3*c3 );
      if( coef>0 )
        coef = sqrt( coef );
      else//problem with this rotation...
      {
        coef = 0;
        Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
        quat_delta.normalize();
        c1=quat_delta.x(); c2=quat_delta.y(); c3=quat_delta.z();
        coef = quat_delta.w();
      }

      Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
      Eigen::Quaterniond rot_total = quat_delta * rot_init;
      //add camera parameters to p:
      rotations_[ idx_cam ] = rot_total.toRotationMatrix();

      translations_[ idx_cam ](0) += p_local[3];
      translations_[ idx_cam ](1) += p_local[4];
      translations_[ idx_cam ](2) += p_local[5];

      //update camera's structure:
      cv::Mat newRotation,newTranslation;
      cv::eigen2cv( rotations_[ idx_cam ], newRotation );
      cv::eigen2cv( translations_[ idx_cam ], newTranslation );
      cameras_[ idx_cam ].setRotationMatrix( newRotation );
      cameras_[ idx_cam ].setTranslationVector( newTranslation );

      p_local+=cnp;
    }
    for ( j = 0; j < point_computed_.size(); ++j )
    {//for each 3D point:
      if( pointOK[j])
      {
        cv::Vec3d cv3DPoint;
        cv3DPoint[ 0 ] = *(p_local++);
        cv3DPoint[ 1 ] = *(p_local++);
        cv3DPoint[ 2 ] = *(p_local++);
        point_computed_[ j ].set3DPosition( cv3DPoint );
      }
    }

    //////////////////////////////////////////////////////////////////////////
#ifdef PRINT_DEBUG
    //Debug compare projected point vs estimated point:
    idx_visible = 0;
    //2D projected points
    double max_distance_1 = 0;
    double max_depth1 = 0;
    j_real = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      if( pointOK[j] )
      {
      for ( i=0; i < m; ++i )
      {//for each camera:
        if( vmask[ VISIB_MASK_new ] )
        {
          cv::Vec3d& cv3DPoint = point_computed_[ j ];
          //cout<<"Vec3d : "<< cv3DPoint[ 0 ]<<", "<< cv3DPoint[ 1 ]<<", "<< cv3DPoint[ 2 ]<<endl;
          int idx_cam = idx_cameras[i];
          cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
            point_computed_[ j ].getPointIndex( idx_cam ) );
          cv::Vec2d proj = cameras_[ idx_cam ].project3DPointIntoImage(cv3DPoint);/*
          cout<<pt.pt.x<<","<<pt.pt.y<<" -> ";
          cout<<proj[0]<<","<<proj[1]<<endl;*/
          max_distance_1 += (pt.pt.x - proj[0])*(pt.pt.x - proj[0]) +
            (pt.pt.y - proj[1])*(pt.pt.y - proj[1]);
          libmv::Vec3 X(cv3DPoint[0],cv3DPoint[1],cv3DPoint[2]);
          max_depth1 += abs( (rotations_[ idx_cam ]*X)(2) + translations_[ idx_cam ](2) );
        }
      }
      j_real++;
    }
      //system("pause");
    }
    cout<< (max_distance)<<"  ; "<<(max_distance_1)<<endl;
    cout<< (max_depth)<<"  ; "<<(max_depth1)<<endl;
#endif
    ////////////////////////////////////////////////////////////////////////*/

    }

    delete [] vmask;//visibility mask
    delete [] p;//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    delete [] x;// measurement vector
  }

  bool EuclideanEstimator::cameraResection( unsigned int image )
  {
    //wrap the lourakis SBA:
    cout<<"resection"<<endl;
    unsigned int n = point_computed_.size( ),   // number of points
      m = 0,   // number of images (or camera)
      mcon = 0,// number of images (starting from the 1st) whose parameters should not be modified.
      cnp = 6,// number of parameters for ONE camera; e.g. 6 for Euclidean cameras
      //use only vector part of quaternion to enforce the unit lenght...
      mnp = 2;// number of parameters for ONE projected point; e.g. 2 for Euclidean points

    unsigned int i = 0, j = 0,
      nb_cam = camera_computed_.size( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );
    vector< TrackOfPoints > real_track;
    //keep only tracks having image:

    for(i=0; i<n; i++)
      if( point_computed_[i].containImage(image) )
        real_track.push_back(point_computed_[i]);
    n = real_track.size();

    //now for fun show the sequence on images:
    //sequence_.showTracks( image, real_track );

    //because some points are sometime not visible:
    vector<int> idx_cameras;
    libmv::vector< libmv::Mat3 > intra_p;
    int nz_count = 0;
    for ( i = 0; i < nb_cam; ++i )
    {//for each camera:
      if( camera_computed_[ i ] && i!=image )
      {
        idx_cameras.push_back(i);
        m++;//increament of camera count

        int nb_projection = 0;
        for ( j = 0; j < n; ++j )
        {//for each 3D point:
          if( real_track[ j ].containImage( i ) )
            nb_projection++;
        }
        nz_count += nb_projection;
      }
    }

    mcon = m;//other cameras are constant!

    idx_cameras.push_back(image);
    m++;//increament of camera count

    int nb_projection = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      if( real_track[ j ].containImage( image ) )
        nb_projection++;
    }
    nz_count += nb_projection;

    //2D points:
    char *vmask = new char[ n*m ];//visibility mask: vmask[i, j]=1 if point i visible in image j, 0 otherwise.
    double *p = new double[m*cnp + n*3];//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    // aj are the image j parameters, bi are the i-th point parameters

    double *x = new double[ 2*nz_count ];// measurements vector: (x_11^T, .. x_1m^T, ..., x_n1^T, .. x_nm^T)^T where
    // x_ij is the projection of the i-th point on the j-th image.
    // NOTE: some of the x_ij might be missing, if point i is not visible in image j;
    // see vmask[i, j], max. size n*m*mnp

    libmv::vector< Eigen::Quaterniond > init_rotation;
    libmv::vector< libmv::Vec3 > init_translat;
    //update each variable:
    double *p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      intra_p.push_back( intra_params_[idx_cam] );
      //extrinsic parameters only (intra are know in euclidean reconstruction)
      init_rotation.push_back( (Eigen::Quaterniond)rotations_[ idx_cam ] );
      init_translat.push_back( translations_[ idx_cam ] );
      //add camera parameters to p:
      //as this is rotation, the quaternion's length is unity. Only 3 values are needed.
      //4th value equal:
      //sqrt(1.0 - quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2]));

      p_local[0] = 0; p_local[1] = 0; p_local[2] = 0;


      p_local[3] = 0; p_local[4] = 0; p_local[5] = 0;

      p_local+=cnp;
    }

    //now add the projections and 3D points:
    unsigned int idx_visible = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      for ( i=0; i < m; ++i )
      {//for each camera:
        int idx_cam = idx_cameras[i];
        vmask[ VISIB_MASK ] = real_track[ j ].containImage( idx_cam );
        if( vmask[ VISIB_MASK ] )
        {
          cv::KeyPoint pt = points_to_track[ idx_cam ]->getKeypoint(
            real_track[ j ].getPointIndex( idx_cam ) );
          x[ idx_visible++ ] = pt.pt.x;
          x[ idx_visible++ ] = pt.pt.y;
        }
      }
    }
    double* points3D_values = p_local;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      cv::Vec3d cv3DPoint = real_track[ j ];
      *(p_local++) = cv3DPoint[ 0 ];
      *(p_local++) = cv3DPoint[ 1 ];
      *(p_local++) = cv3DPoint[ 2 ];
    }

    //TUNING PARAMETERS:
    int itmax = 1000;        //max iterations
    int verbose = 0;         //no debug
    double opts[SBA_OPTSSZ] = {
      0.1,		//Tau
      1e-12,		//E1
      1e-12,		//E2
      0,		//E3 average reprojection error
      0		//E4 relative reduction in the RMS reprojection error
    };

    double info[SBA_INFOSZ];
    bundle_datas data(intra_p,init_rotation, init_translat,
      cnp, 3, mnp, 0, mcon);
    data.points3D = points3D_values;
    data.initial_projections = x;
    //use sba library
    int iter = sba_mot_levmar_x(n, m, mcon, vmask, p, cnp, x, NULL, mnp,
      img_projsRT_x, NULL, (void*)&data, itmax, 0, opts, info);

    bool resection_ok = true;
    if( ( iter<=0 ) || (info[1]/nz_count)>100 )
    {
      resection_ok = false;
      std::cout<<"resection rejected ("<<nz_count<<") : "<<info[1]/nz_count<<std::endl;
    }
    else
    {
      std::cout<<"SBA returned in "<<iter<<" iter, reason "<<info[6]
      <<", error "<<info[1]/nz_count<<" [initial "<< info[0]/nz_count<<"]\n";


      //set new values:
      m = idx_cameras.size();
      n = real_track.size( );
      idx_visible = 0;
      p_local = p + cnp*(m-1);

      //extrinsic parameters only (intra are know in euclidean reconstruction)
      Eigen::Quaterniond rot_init = data.rotations[ m-1 ];
      double c1 = p_local[0];
      double c2 = p_local[1];
      double c3 = p_local[2];
      double coef=(1.0 - c1*c1 - c2*c2 - c3*c3 );
      if( coef>0 )
        coef = sqrt( coef );
      else//problem with this rotation...
      {
        coef = 0;
        Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
        quat_delta.normalize();
        c1=quat_delta.x(); c2=quat_delta.y(); c3=quat_delta.z();
        coef = quat_delta.w();
      }

      Eigen::Quaterniond quat_delta( coef, c1, c2, c3 );
      Eigen::Quaterniond rot_total = quat_delta * rot_init;
      //add camera parameters to p:
      rotations_[ image ] = rot_total.toRotationMatrix();

      translations_[ image ](0) += p_local[3];
      translations_[ image ](1) += p_local[4];
      translations_[ image ](2) += p_local[5];

      //update camera's structure:
      cv::Mat newRotation,newTranslation;
      cv::eigen2cv( rotations_[ image ], newRotation );
      cv::eigen2cv( translations_[ image ], newTranslation );
      cameras_[ image ].setRotationMatrix( newRotation );
      cameras_[ image ].setTranslationVector( newTranslation );

      camera_computed_[ image ] = true;
    }

    delete [] vmask;//visibility mask
    delete [] p;//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    delete [] x;// measurement vector


    return resection_ok;
  }

  void EuclideanEstimator::initialReconstruction( vector<TrackOfPoints>& tracks,
    int image1, int image2 )
  {
    camera_computed_[ image1 ] = true;
    index_origin = image1;

    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );
    libmv::Mat3 E;
    Ptr<PointsToTrack> point_img1 = points_to_track[ image1 ];
    Ptr<PointsToTrack> point_img2 = points_to_track[ image2 ];
    //first extract points matches:
    libmv::Mat2X x1,x2;
    //for each points:
    unsigned int key_size = tracks.size( );
    unsigned int i;
    vector<TrackOfPoints> matches;

    for ( i=0; i < key_size; ++i )
    {
      TrackOfPoints &track = tracks[ i ];
      if( track.containImage( image1 ) && track.containImage( image2 ) )
        matches.push_back( track );
    }
    x1.resize( 2,matches.size( ) );
    x2.resize( 2,matches.size( ) );

    key_size = matches.size( );
    vector<cv::Vec2d> pointImg1,pointImg2;
    for ( i=0; i < key_size; ++i )
    {
      TrackOfPoints &track = matches[ i ];
      cv::DMatch match = track.toDMatch( image1, image2 );

      pointImg1.push_back( cv::Vec2d( point_img1->getKeypoint( match.trainIdx ).pt.x,
        point_img1->getKeypoint( match.trainIdx ).pt.y ) );
      pointImg2.push_back( cv::Vec2d( point_img2->getKeypoint( match.queryIdx ).pt.x,
        point_img2->getKeypoint( match.queryIdx ).pt.y ) );
    }
    vector<cv::Vec2d> pointNorm1 = cameras_[ image1 ].getIntraParameters( )->
      pixelToNormImageCoordinates( pointImg1 );
    vector<cv::Vec2d> pointNorm2 = cameras_[ image2 ].getIntraParameters( )->
      pixelToNormImageCoordinates( pointImg2 );
    key_size = pointNorm1.size( );
    for ( i=0; i < key_size; ++i )
    {
      x1( 0,i ) = -pointNorm1[ i ][ 0 ];
      x1( 1,i ) = -pointNorm1[ i ][ 1 ];
      x2( 0,i ) = -pointNorm2[ i ][ 0 ];
      x2( 1,i ) = -pointNorm2[ i ][ 1 ];
    }
    
    double error = robust5Points( x1, x2,
      intra_params_[ image1 ], intra_params_[ image2 ], E );


    //std::cout<<"E: "<<E<<std::endl;
    //std::cout<<"max_error: "<<error<<std::endl;


    //From this essential matrix extract relative motion:
    libmv::Mat3 R;
    libmv::Vec3 t;
    libmv::Vec2 x1Col, x2Col;
    x1Col << x1( 0,0 ), x1( 1,0 );
    x2Col << x2( 0,0 ), x2( 1,0 );
    bool ok = libmv::MotionFromEssentialAndCorrespondence( E,
      intra_params_[ image1 ], x1Col,
      intra_params_[ image2 ], x2Col,
      &R, &t );

    rotations_[ image2 ] = R * rotations_[ image1 ];
    translations_[ image2 ] = t + R * translations_[ image1 ];

    //update camera's structure:
    cv::Mat newRotation,newTranslation;
    cv::eigen2cv( rotations_[ image2 ], newRotation );
    cv::eigen2cv( translations_[ image2 ], newTranslation );
    cameras_[ image2 ].setRotationMatrix( newRotation );
    cameras_[ image2 ].setTranslationVector( newTranslation );

    //this camera is now computed:
    camera_computed_[ image2 ] = true;

    //Triangulate the points:
    StructureEstimator se( &sequence_, &this->cameras_ );
    vector<int> images_to_compute;
    images_to_compute.push_back( image1 );
    images_to_compute.push_back( image2 );
    point_computed_ = se.computeStructure( images_to_compute );
    //bundleAdjustement();
  }

  void EuclideanEstimator::computeReconstruction( )
  {
    vector<TrackOfPoints>& tracks = sequence_.getTracks( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence_.getPoints( );
    ImagesGraphConnection &images_graph = sequence_.getImgGraph( );
    //double ransac_threshold = 0.4 * sequence_.getImage( 0 ).rows / 100.0;
    double ransac_threshold = 3.0;
    //now create the graph:

    int img1,img2;
    int nbMatches = images_graph.getHighestLink( img1,img2 );
    vector<ImageLink> bestMatches;
    images_graph.getOrderedLinks( bestMatches, MIN(nbMatches/2,100), nbMatches );
    double min_inliners=1e7;
    int index_of_min=0;
    cv::Mat minFundamental;
    for( unsigned int cpt=0;cpt<bestMatches.size( );cpt++ )
    {
      //construct the homography and choose the worse matches:
      //( see Snavely "Modeling the World from Internet Photo Collections" )
      std::vector<cv::Point2f> pointsImg1, pointsImg2;
      vector<uchar> status;
      points_to_track[ bestMatches[ cpt ].imgSrc ]->getKeyMatches( tracks,
        bestMatches[ cpt ].imgDest, pointsImg1 );
      points_to_track[ bestMatches[ cpt ].imgDest ]->getKeyMatches( tracks,
        bestMatches[ cpt ].imgSrc, pointsImg2 );

      //compute the homography:
      cv::findHomography( pointsImg1,pointsImg2,status,CV_RANSAC,
        ransac_threshold );
      //count the inliner points:
      double inliners=0;
      for( unsigned int i=0;i<status.size( );++i )
      {
        if( status[ i ] != 0 )
          inliners++;
      }
      double percent_inliner = inliners/static_cast<double>( pointsImg1.size( ) );
      if( percent_inliner < min_inliners )
      {
        min_inliners = percent_inliner;
        index_of_min = cpt;
        minFundamental = cv::findFundamentalMat( pointsImg1, pointsImg2,
          status, cv::FM_RANSAC );
      }
    }
    //we will start the reconstruction using bestMatches[ index_of_min ]
    //to avoid degenerate cases such as coincident cameras
    img1 = bestMatches[ index_of_min ].imgSrc;
    img2 = bestMatches[ index_of_min ].imgDest;
    cout<<img1<<", "<<img2<<endl;
    //sequence_.showTracksBetween(img1, img2);

    vector<int> images_computed;
    images_computed.push_back( img1 );
    images_computed.push_back( img2 );
    camera_computed_[ img1 ] = true;
    initialReconstruction( tracks, img1, img2 );
    //bundleAdjustement();

    //now we have updated the position of the camera which take img2
    //and 3D estimation from these 2 first cameras...
    //Find for other cameras position:
    vector<ImageLink> images_close;
    int nbIter = 0 ;
    //while( nbMatches>10 && images_computed.size()<cameras_.size()/2 && nbIter<4 )
    {
      nbIter++;
      images_close.clear( );
      while ( images_close.size( ) < 2 )
      {
        for(size_t cpt = 0; cpt<images_computed.size(); ++cpt )
        {
          images_graph.getImagesRelatedTo( images_computed[ cpt ],
            images_close, nbMatches * 0.8 - 10 );
        }
        nbMatches = nbMatches * 0.8 - 10;
      }

      //for each images, comptute the camera position:
      for( size_t cpt=0;cpt<images_close.size( );cpt++ )
      {
        //We don't want to compute twice the same camera position:
        int new_id_image = -1,
          old_id_image = -1;
        if( !camera_computed_[ images_close[ cpt ].imgSrc ] )
        {
          new_id_image = images_close[ cpt ].imgSrc;
          old_id_image = images_close[ cpt ].imgDest;
        }
        else
        {
          new_id_image = images_close[ cpt ].imgDest;
          old_id_image = images_close[ cpt ].imgSrc;
        }

        if( new_id_image >= 0 )
        {
          if( cameraResection( new_id_image ) )
            images_computed.push_back( new_id_image );
        }
      }

      //Triangulate the points:
      StructureEstimator se( &sequence_, &this->cameras_ );

      vector<int> images_to_compute;
      unsigned int key_size = camera_computed_.size( );
      for (unsigned int i=0; i < key_size; ++i )
      {
        if( camera_computed_[ i ] )
          images_to_compute.push_back( i );
      }

      point_computed_ = se.computeStructure( images_to_compute, 2 );
      se.removeOutliersTracks( 5, &point_computed_ );
      SequenceAnalyzer::keepOnlyCorrectMatches(sequence_,2,0);
      // Performs a bundle adjustment
      //bundleAdjustement();
    }//*/
    bundleAdjustement();
    viewEstimation();
  }

  void EuclideanEstimator::viewEstimation()
  {
    vector<cv::Vec3d> tracks3D;
    vector< unsigned int > colors;
    vector<TrackOfPoints>::iterator itTrack=point_computed_.begin( );
    while ( itTrack != point_computed_.end( ) )
    {
      tracks3D.push_back( ( cv::Vec3d )( *itTrack ) );
      colors.push_back( itTrack->getColor() );
      itTrack++;
    }

    //////////////////////////////////////////////////////////////////////////
    // Open 3D viewer and add point cloud
    Visualizer debugView ( "Debug viewer" );
    debugView.add3DPointsColored( tracks3D,colors, "Euclidean estimated" );
    
    for( unsigned int i = 0; i<cameras_.size( ) ; ++i )
      if( camera_computed_[i] )
      {
        std::stringstream cam_name;
        cam_name<<"Cam"<< ( i+1 );
        debugView.addCamera( cameras_[ i ],
          cam_name.str() );
      }//*/
      


      debugView.runInteract( );
  }
}
