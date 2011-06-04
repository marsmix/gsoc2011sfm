/**********************************************************\
|                   libmv core functions                   |
\**********************************************************/
//Added _CORE_ in define to remind us it's not libmv full library.
#ifndef LIBMV_CORE_AUTOCALIBRATION_ESTIMATOR_H
#define LIBMV_CORE_AUTOCALIBRATION_ESTIMATOR_H

#include "../numeric/libmv_types.h"
#include "../numeric/conditioning.h"
//EssentialFromRt need RelativeCameraMotion, so:
#include "../estimator/projection_estimator.h"

#include <vector>

namespace libmv
{
  namespace autocalibration
  {
    // Create intra parameters using absolute conic:
    void K_From_AbsoluteConic(const Mat3 &W, Mat3 *K);
    //class to perform linear estimation of metric transformation H:
    //If {P, X} is a projective reconstruction, then {P H, H^{-1} X} is
    // a metric reconstruction.
    /** \brief Compute a metric reconstruction from a projective one by computing
    *         the dual absolute quadric using linear constraints.
    *         
    * We follow the linear approach proposed by Pollefeys in section 3.4 of [1]
    *
    * [1] M. Pollefeys, L. Van Gool, M. Vergauwen, F. Verbiest, K. Cornelis,
    *     J. Tops, R. Koch, "Visual modeling with a hand-held camera",
    *     International Journal of Computer Vision 59(3), 207-232, 2004.
    */
    class AutoCalibrationLinear {
    public:
      /** \brief Add a projection to be used for autocalibration.
      *
      *  \param P The projection matrix.
      *  \param width  The width of the image plane.
      *  \param height The height of the image plane.
      *
      *  The width and height parameters are used to normalize the projection
      *  matrix for improving numerical stability.  The don't need to be exact.
      */
      int AddProjection(const Mat34 &P, double width, double height);

      /** \brief Computes the metric updating transformation.
      *
      *  \return The homography, H, that transforms the space into a metric space.
      *          If {P, X} is a projective reconstruction, then {P H, H^{-1} X} is
      *          a metric reconstruction.  Note that this follows the notation of
      *          HZ section 19.1 page 459, and not the notation of Pollefeys'
      *          paper [1].
      */
      Mat4 MetricTransformation();

    private:
      /** \brief Add constraints on the absolute quadric based assumptions on the
      *         parameters of one camera.
      *
      *  \param P The projection matrix of the camera in projective coordinates.
      */
      void AddProjectionConstraints(const Mat34 &P);

      /** \brief Computes the constraint associated to elements of the DIAC.
      *
      *  \param P The projection used to project the absolute quadric.
      *  \param i Row of the DIAC.
      *  \param j Column of the DIAC.
      *  \return The coeficients of the element i, j of the dual image of the
      *          absolute conic when written as a linear combination of the
      *          elements of the absolute quadric.  There are 10 coeficients since
      *          the absolute quadric is represented by 10 numbers.
      */
      static Vec wc(const Mat34 &P, int i, int j);

      static Mat4 AbsoluteQuadricMatFromVec(const Vec &q);

      static void NormalizeProjection(const Mat34 &P,
        double width, double height, Mat34 *P_new);

      static void DenormalizeProjection(const Mat34 &P,
        double width, double height, Mat34 *P_new);

    private:
      vector<Mat34> projections_; // The *normalized* projection matrices.
      vector<double> widths_;
      vector<double> heights_;
      vector<Vec> constraints_;  // Linear constraints on q.
    };
  }
}

#endif