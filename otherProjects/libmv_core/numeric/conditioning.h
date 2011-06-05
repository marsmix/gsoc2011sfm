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
|                   libmv core functions                   |
\**********************************************************/
//Added _CORE_ in define to remind us it's not libmv full library.
#ifndef LIBMV_CORE_MULTIVIEW_CONDITIONNING_H_
#define LIBMV_CORE_MULTIVIEW_CONDITIONNING_H_

//Here is a change from classic libmv:
//In order to have something which tell us this is
//the core libmv library, numerical.h is now renamed to
//libmv_types.h *but the content is not modified*
#include "libmv_types.h"

namespace libmv {

// Point conditioning (non isotropic)
void _LIBMV_DLL_ PreconditionerFromPoints(const Mat &points, Mat3 *T);
// Point conditioning (isotropic)
void _LIBMV_DLL_ IsotropicPreconditionerFromPoints(const Mat &points, Mat3 *T);

void _LIBMV_DLL_ ApplyTransformationToPoints(const Mat &points,
                                 const Mat3 &T,
                                 Mat *transformed_points);

void _LIBMV_DLL_ NormalizePoints(const Mat &points,
                     Mat *normalized_points,
                     Mat3 *T);

void _LIBMV_DLL_ NormalizeIsotropicPoints(const Mat &points,
                              Mat *normalized_points,
                              Mat3 *T);

/// Use inverse for unnormalize
struct UnnormalizerI {
  // Denormalize the results. See HZ page 109.
  static void _LIBMV_DLL_ Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H);
};

/// Use transpose for unnormalize
struct UnnormalizerT {
  // Denormalize the results. See HZ page 109.
  static void _LIBMV_DLL_ Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H);
};

} //namespace libmv


#endif // LIBMV_MULTIVIEW_CONDITIONNING_H_
