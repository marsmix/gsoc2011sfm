// Copyright (c) 2009 libmv authors.
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

#ifndef LIBMV_MULTIVIEW_N_VIEW_KERNEL_H_
#define LIBMV_MULTIVIEW_N_VIEW_KERNEL_H_

#include "libmv/base/vector.h"
#include "libmv/logging/logging.h"
#include "libmv/multiview/conditioning.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
  namespace n_view {
    namespace kernel {

      // I try to do this the same way than two_view::kernel::Kernel.
      // This is one example (targeted at solvers that operate on correspondences
      // between views) that shows the "kernel" part of a robust fitting
      // problem:
      //
      //   1. The model; Mat3 in the case of the F or H matrix.
      //      In the case of triangulation, it's Matrix<T, 4, 1> (the 3D point)
      //   2. The minimum number of samples needed to fit; 7, 8 (or 2).
      //   3. The minimum number of views to fit; (usually 2).
      //   4. A way to convert samples and an optional model to a result
      //   5. A way to convert a sample, a model and an estimate to an error.
      //
      // Of particular note is that the kernel does not expose what the samples are.
      // All the robust fitting algorithm sees is that there is some number of
      // samples; it is able to fit subsets of them (via the kernel) and check their
      // error, but can never access the samples themselves.
      //
      // The Kernel objects must follow the following concept so that the robust
      // fitting alogrithm can fit this type of relation:
      //
      //   1. Kernel::Model
      //   2. Kernel::MINIMUM_SAMPLES
      //   3. Kernel::MINIMUM_VIEWS
      //   4. Kernel::Fit(vector<int>, vector<Kernel::Model> *)
      //   5. Kernel::Error(int, Model) -> error
      //
      // The fit routine must not clear existing entries in the vector of models; it
      // should append new solutions to the end.
      template<typename SolverArg,
        typename ErrorArg,
        typename ModelArg = Mat,
        typename EstimationArg = Mat3>
      class Kernel {
      public:
        Kernel(const vector<Mat> &x, const vector<ModelArg> &model)
          : x_(x), models_(model) {};
        typedef SolverArg Solver;
        typedef ModelArg  Model;
        typedef EstimationArg  EstimClass;
        enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
        enum { MINIMUM_VIEWS = Solver::MINIMUM_VIEWS };
        void Fit(const vector<int> &samples, vector<EstimClass> *estimate) const {
          vector<Mat> x_selected;
          unsigned int i=0;
          for (unsigned int i=0; i<x_.size(); ++i)
          {
            x_selected.push_back( ExtractColumns( x_[i], samples ) );
          }
          Solver::Solve( x_selected, models_, estimate );
        }
        inline double Error(int sample, const Model &model) const {
          vector<Vec> x_selected;
          unsigned int i=0;
          for (unsigned int i=0; i<x_.size(); ++i)
          {
            x_selected.push_back( static_cast<Vec>(x_[i].col(sample)) );
          }
          return ErrorArg::Error(model, 
            x_selected,
            models_);
        }
        int NumSamples() const {
          return x1_.cols();
        }
        static void Solve(const vector<Mat> &x, const vector<ModelArg> &models,
          vector<EstimClass> *estimates) {
          // By offering this, Kernel types can be passed to templates.
          Solver::Solve(x, models, estimates);
        }
      protected:
        const vector<Mat> &x_;
        const vector<ModelArg> &models_;
      };

    }  // namespace kernel
  }  // namespace two_view
}  // namespace libmv

#endif  // LIBMV_MULTIVIEW_TWO_VIEW_KERNEL_H_
