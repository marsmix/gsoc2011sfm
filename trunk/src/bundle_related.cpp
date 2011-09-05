
#include "bundle_related.h"

#include "opencv2/core/eigen.hpp"

#include "SequenceAnalyzer.h"
#include "CameraPinholeDistor.h"
#include "PointsToTrack.h"
#include "PointOfView.h"


namespace OpencvSfM{

  using std::vector;
  using cv::Ptr;


  //TODO: intra parameters are badly integrated:
  //1) when multiple cameras share the same intra parameters, computation is not optimized
  //2) Skew and ratio of focal are not correctly estimated... See how this can be improved!
  void full_bundle( SequenceAnalyzer &sequence,
    std::vector<PointOfView>& cameras)
  {
    //wrap the lourakis SBA...
    //everything should be adjusted!
    std::vector< TrackOfPoints > point_computed_ =
      sequence.getTracks();
    //what should we compute?
    unsigned int n = point_computed_.size( ),   // number of points
      ncon = 0,// number of points (starting from the 1st) whose parameters should not be modified.
      m = 0,   // number of images (or camera)
      mcon = 0,// number of cameras (starting from the 1st) whose parameters should not be modified.
      cnp = cameras[0].getNbMissingParams(),// number of parameters for ONE camera;
      pnp = 3,// number of parameters for ONE 3D point; e.g. 3 for Euclidean points
      mnp = 2;// number of parameters for ONE projected point; e.g. 2 for Euclidean points

    unsigned int i = 0, j = 0,
      nb_cam = cameras.size( );
    vector< Ptr< PointsToTrack > > &points_to_track = sequence.getPoints( );

    //because some points are sometime not visible:
    vector<int> idx_cameras;

    std::vector<bool> pointOK;
    int nbPoints = 0;
    for ( j = 0; j < n; ++j )
    {//for each 3D point:
      //test if at least 2 views see this point:
      int nbCam = 0;
      for(size_t k =0; k<nb_cam; ++k)
      {
        if( point_computed_[ j ].containImage( k ) )
          nbCam++;
      }
      pointOK.push_back( nbCam>=2 );
      if( pointOK[j] )
        nbPoints++;
    }

    int nz_count = 0;
    for ( i = 0; i < nb_cam; ++i )
    {//for each camera:

      int nb_projection = 0;
      for ( j = 0; j < n; ++j )
      {//for each 3D point:
        if( pointOK[j])
        {
          if( point_computed_[ j ].containImage( i ) )
            nb_projection++;
        }
      }
      if( nb_projection>30 )//a camera should see at least 10 points
      {
        nz_count += nb_projection;
        idx_cameras.push_back(i);
        m++;//increament of camera count
      }
      else
        std::clog<<"remove camera "<<i<<" from bundle"<<std::endl;
    }
    n=nbPoints;
    nb_cam = m;

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
    libmv::vector< libmv::Mat3 > intra_p;
    libmv::vector< int > idx_intra;
    //update each variable:
    int idx_visible = 0;
    double *p_local = p;
    for ( i=0; i < m; ++i )
    {//for each camera:
      int idx_cam = idx_cameras[i];
      libmv::Mat3 intra_param, rotation_mat;
      libmv::Vec3 translation_vec;
      cv::Ptr<Camera> intra=cameras[ idx_cam ].getIntraParameters( );
      //is this camera shared?
      const std::vector<PointOfView*> & relV = intra->getRelatedViews();

      int idx_cam_intra = -1;
      for(size_t cpt = 0; cpt<relV.size() && idx_cam_intra<0; cpt++)
      {
        for(int cpt1 = 0; (cpt1<intra_p.size()) && (idx_cam_intra<0); cpt1++)
        {
          int idx_cam_bis = idx_intra[ cpt1 ];
          if ( idx_cam_bis!=idx_cam && (&cameras[ idx_cam_bis ]) == relV[ cpt ] )
          {
            std::clog<<"intra params for camera "<< i <<
              " are the same params than camera: "<<idx_cam_bis<<std::endl;
            idx_cam_intra = idx_cam_bis;
          }
        }
      }

      cv::cv2eigen( intra->getIntraMatrix( ).t(), intra_param );
      if( idx_cam_intra<0 )
      {
        idx_intra.push_back( intra_p.size() );
        intra_p.push_back( intra_param );
      }
      else
      {//as this intra parameter is shared, it will not being updated during bundle!
        idx_intra.push_back( idx_cam_intra );
      }

      cv::cv2eigen( cameras[ idx_cam ].getRotationMatrix( ), rotation_mat );
      cv::cv2eigen( cameras[ idx_cam ].getTranslationVector( ), translation_vec );

      init_rotation.push_back( (Eigen::Quaterniond)rotation_mat );
      init_translat.push_back( translation_vec );

      for(size_t i=0; i<cnp; ++i)
        p_local[i] = 0;

      p_local[3] = intra_param(1,1) / intra_param(0,0);

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
          vmask[ i+j_real*m ] = point_computed_[ j ].containImage( idx_cam );
          if( vmask[ i+j_real*m ] )
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

    bundle_datas data(idx_intra,intra_p,init_rotation,init_translat,
      cnp, pnp, mnp,ncon, mcon);
    data.points3D = points3D_values;

    //TUNING PARAMETERS:
    int itmax = 10000;        //max iterations
    int verbose = 1;
    double opts[SBA_OPTSSZ] = {
      0.1,		//Tau
      1e-20,		//E1
      1e-20,		//E2
      0,		//E3 average reprojection error
      0		//E4 relative reduction in the RMS reprojection error
    };

    double info[SBA_INFOSZ];

    //use sba library
    int iter = sba_motstr_levmar_x(n, ncon, m, mcon, vmask, p, cnp, pnp, x, NULL, mnp,
        img_projsKRTS_x, NULL, (void*)&data, itmax, 0, opts, info);
    std::cout<<"SBA ("<<nz_count<<") returned in "<<iter<<" iter, reason "<<info[6]
    <<", error "<<info[1]<<" [initial "<< info[0]<<"]\n";
    if(iter>1)
    {
      //set new values:
      idx_visible = 0;
      p_local = p;
      for ( i=0; i < m; ++i )
      {//for each camera:
        int idx_cam = idx_cameras[i];
        //intra parameters first:
        libmv::Mat3 rotation_mat;
        libmv::Vec3 translation_vec;
        cv::Ptr<Camera> intra=cameras[ idx_cam ].getIntraParameters( );
        cv::cv2eigen( cameras[ idx_cam ].getRotationMatrix( ), rotation_mat );
        cv::cv2eigen( cameras[ idx_cam ].getTranslationVector( ), translation_vec );

        int idx_intra = data.idx[i];
        libmv::Mat3& K = data.intraParams[ idx_intra ];
        double* pIntra=p+idx_intra*cnp;
        double Kparms[] = {K( 0,0 ) + pIntra[0], K( 2,0 ) + pIntra[1],
          K( 2,1 ) + pIntra[2], pIntra[3], K( 1,0 ) + pIntra[4] };
        /*
        std::cout<<std::endl<<"old intra : "<<intra->getIntraMatrix()<<std::endl;
        std::cout<<idx_intra<<"; "<<
          pIntra[0]<<" "<<pIntra[1]<<" "<<pIntra[2]<<" "<<pIntra[3]<<std::endl;
        intra->updateIntrinsic( Kparms, 5, false );
        std::cout<<"new intra : "<<intra->getIntraMatrix()<<std::endl;*/

        Eigen::Quaterniond rot_init = data.rotations[i];
        double c1 = p_local[5];
        double c2 = p_local[6];
        double c3 = p_local[7];
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
        rotation_mat = rot_total.toRotationMatrix();

        translation_vec(0) += p_local[8];
        translation_vec(1) += p_local[9];
        translation_vec(2) += p_local[10];

        //update camera's structure:
        cv::Mat newRotation,newTranslation;
        cv::eigen2cv( rotation_mat, newRotation );
        cv::eigen2cv( translation_vec, newTranslation );
        cameras[ idx_cam ].setRotationMatrix( newRotation );
        cameras[ idx_cam ].setTranslationVector( newTranslation );

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

    }

    delete [] vmask;//visibility mask
    delete [] p;//initial parameter vector p0: (a1, ..., am, b1, ..., bn).
    delete [] x;// measurement vector
  }

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

      int idx_intra = datas->idx[j];
      libmv::Mat3& K = datas->intraParams[ idx_intra ];
      double Kparms[] = {K( 0,0 ),K( 2,0 ),K( 2,1 ),K( 1,1 )/K( 0,0 ),K( 1,0 )};

      Eigen::Quaterniond rot_init = datas->rotations[j];
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

          calcImgProjFullR(Kparms, trot, trans, ppt, pmeas); // evaluate Q in pmeas
        }
    }
  }

  void img_projsKRTS_x(/*cameras and points*/ double *p,
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

      int idx_intra = datas->idx[j];
      libmv::Mat3& K = datas->intraParams[ idx_intra ];
      double* pIntra=p+idx_intra*cnp;
      double Kparms[] = {K( 0,0 ) + pIntra[0], K( 2,0 ) + pIntra[1],
        K( 2,1 ) + pIntra[2], 1, 0};//pIntra[3], K( 1,0 ) + pIntra[4] };

      Eigen::Quaterniond rot_init = datas->rotations[j];

      // full quat for initial rotation estimate:
      double pr0[] = {rot_init.w(), rot_init.x(), rot_init.y(), rot_init.z()};
      libmv::Mat34 proj;

      pqr=pa + j*cnp + 5;//skip the intra parameters...
      pt=pqr + 3; // quaternion vector part has 3 elements
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

          calcImgProjFullR(Kparms, trot, trans, ppt, pmeas); // evaluate Q in pmeas
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

      int idx_intra = datas->idx[j];
      libmv::Mat3& K = datas->intraParams[ idx_intra ];
      double Kparms[] = {K( 0,0 ),K( 2,0 ),K( 2,1 ),K( 1,1 )/K( 0,0 ),K( 1,0 )};

      Eigen::Quaterniond rot_init = datas->rotations[j];

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

  void img_projsRTS_jac_x(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata)
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

}