// Copyright (c) 2010 libmv authors.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

/**********************************************************\
|             libmv core functions proposition             |
\**********************************************************/
//Added _CORE_ in define to remind us it's not libmv full library.
#ifndef LIBMV_CORE_FUNDAMENTAL_ESTIMATOR_H
#define LIBMV_CORE_FUNDAMENTAL_ESTIMATOR_H

#include "../numeric/libmv_types.h"
#include "../numeric/conditioning.h"
#include "../numeric/poly.h"
#include "../numeric/vector.h"

//as we need essential matrix for FocalReprojectionError,
//we need to include essential_estimator:
#include "../estimator/essential_estimator.h"
//we need also points triangulation:
#include "../estimator/projection_estimator.h"

#include <vector>

namespace libmv
{
    //////////////////////////////////////////////////////////////////////////
    ////////////////  How can we create a fundamental matrix:  ///////////////

    //Create fundamental matrix from two projection matrix:
    void _LIBMV_DLL_ FundamentalFromProjections(const Mat34 &P1, const Mat34 &P2, Mat3 *F);
    //Using Essential matrix and camera intra parameters, compute the fundamental matrix
    void _LIBMV_DLL_ FundamentalFromEssential(const Mat3 &E,const Mat3 &K1,const Mat3 &K2,Mat3 *F);
    //8point algorithm without the rank-2 constraint
    double _LIBMV_DLL_ EightPointSolver(const Mat &x1, const Mat &x2, Mat3 *F);
    //8point algorithm with normalisation of points AND rank-2 constraint.
    double _LIBMV_DLL_ NormalizedEightPointSolver(const Mat &x1, const Mat &x2, Mat3 *F);
    // 7 points (minimal case, points coordinates must be normalized before):
    double _LIBMV_DLL_ FundamentalFrom7CorrespondencesLinear(const Mat &x1,const Mat &x2,std::vector<Mat3> *F);
    // 7 points (points coordinates must be in image space):
    double _LIBMV_DLL_ FundamentalFromCorrespondences7Point(const Mat &x1,const Mat &x2,std::vector<Mat3> *F);

    //////////////////////////////////////////////////////////////////////////
    ///////////////  What can we do with a fundamental matrix:  //////////////

    //rank-2 constraint on f matrix to have 2 points for epipoles
    void _LIBMV_DLL_ EnforceFundamentalRank2Constraint(Mat3 *F);
    //normalize fundamental using Frobenius Norm
    void _LIBMV_DLL_ NormalizeFundamental(const Mat3 &F, Mat3 *F_normalized);
    //Create two projection matrix from fundamental matrix:
    void _LIBMV_DLL_ ProjectionsFromFundamental(const Mat3 &F, Mat34 *P1, Mat34 *P2);
    // Approximation of reprojection error using epipolar line distance extracted from F
    double _LIBMV_DLL_ SampsonDistance2(const Mat &F, const Vec2 &x1, const Vec2 &x2);
    //compute the position of the two epipoles
    void _LIBMV_DLL_ EpipolesFromFundamental(const Mat3 &F, Vec3 *e1, Vec3 *e2);
    // Find focal from fundamental using Hartleys method.
    void _LIBMV_DLL_ FocalFromFundamental(const Mat3 &F,const Vec2 &principal_point1,
      const Vec2 &principal_point2,double *f1,double *f2);
    // Find focal from fundamental using Golden ratio search on reproj error on x1 and x2
    void _LIBMV_DLL_ FocalFromFundamentalExhaustive(const Mat3 &F,
      const Vec2 &principal_point,const Mat2X &x1,const Mat2X &x2,
      double min_focal,double max_focal,int n_samples, double *focal);

    //Class used to compute the reprojection errors when changing focal values
    class _LIBMV_DLL_ FocalReprojectionError
    {
    public:
      FocalReprojectionError(const Mat3 &F,
        const Vec2 &principal_point,
        const Mat2X &x1,
        const Mat2X &x2) 
        : F_(F), principal_point_(principal_point), x1_(x1), x2_(x2)
      {
      }

      double operator()(double focal) const;

    private:
      const Mat3 &F_;
      const Vec2 &principal_point_;
      const Mat2X &x1_;
      const Mat2X &x2_;
    };

    // Given a fundamental matrix of two cameras and their principal points,
    // computes the fundamental matrix corresponding to the same cameras with the
    // principal points shifted to a new location.
    void _LIBMV_DLL_ FundamentalShiftPrincipalPoints(const Mat3 &F,
      const Vec2 &p1, const Vec2 &p1_new,
      const Vec2 &p2, const Vec2 &p2_new,
      Mat3 *F_new);
}

#endif
