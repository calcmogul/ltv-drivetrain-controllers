// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010, 2013 Jitse Niesen <jitse@maths.leeds.ac.uk>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_STEM_FUNCTION
#define EIGEN_STEM_FUNCTION

namespace Eigen {

namespace internal {

/** \brief The exponential function (and its derivatives). */
template <typename Scalar>
Scalar stem_function_exp(Scalar x, int) {
  using std::exp;
  return exp(x);
}

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_STEM_FUNCTION
