/**-------------------------------------------------------------------------
@file	nav_math.h

@brief	General dense matrix routines for the navigation tier

Row-major float matrices with explicit dimensions. These cover the work the
attitude kernel fusion_math.h does not: arbitrary MxN products needed by the
15x15 error covariance propagation in the error-state EKF. The 3x3, vector and
quaternion primitives stay in fusion_math.h; this header depends on string.h
only.

All functions are inline and in the NavMath namespace so the same definitions
can be included by several translation units without a link conflict.

@author	Hoang Nguyen Hoan
@date	Jun. 24, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __NAV_MATH_H__
#define __NAV_MATH_H__

#include <string.h>

/** @addtogroup AHRS
  * @{
  */

namespace NavMath {

/// Set an n element array to zero.
inline void Zero(float *A, int n)
{
	memset(A, 0, (size_t)n * sizeof(float));
}

/// Copy n elements.
inline void Copy(float *dst, const float *src, int n)
{
	memcpy(dst, src, (size_t)n * sizeof(float));
}

/// Set a dim x dim matrix to identity.
inline void Eye(float *A, int dim)
{
	Zero(A, dim * dim);
	for (int i = 0; i < dim; i++) {
		A[i * dim + i] = 1.0f;
	}
}

/// C = A * B. A is m x k, B is k x n, C is m x n. C must not alias A or B.
inline void Mul(const float *A, const float *B, float *C, int m, int k, int n)
{
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			float s = 0.0f;
			const float *ar = A + i * k;
			for (int p = 0; p < k; p++) {
				s += ar[p] * B[p * n + j];
			}
			C[i * n + j] = s;
		}
	}
}

/// C = A * B^T. A is m x k, B is n x k, C is m x n. C must not alias A or B.
inline void MulABt(const float *A, const float *B, float *C, int m, int k, int n)
{
	for (int i = 0; i < m; i++) {
		const float *ar = A + i * k;
		for (int j = 0; j < n; j++) {
			const float *br = B + j * k;
			float s = 0.0f;
			for (int p = 0; p < k; p++) {
				s += ar[p] * br[p];
			}
			C[i * n + j] = s;
		}
	}
}

/// A += B, element wise, n elements.
inline void AddEq(float *A, const float *B, int n)
{
	for (int i = 0; i < n; i++) {
		A[i] += B[i];
	}
}

/// A -= B, element wise, n elements.
inline void SubEq(float *A, const float *B, int n)
{
	for (int i = 0; i < n; i++) {
		A[i] -= B[i];
	}
}

/// Force a square dim x dim matrix to be exactly symmetric, in place.
inline void Symmetrize(float *A, int dim)
{
	for (int i = 0; i < dim; i++) {
		for (int j = i + 1; j < dim; j++) {
			float a = 0.5f * (A[i * dim + j] + A[j * dim + i]);
			A[i * dim + j] = a;
			A[j * dim + i] = a;
		}
	}
}

} // namespace NavMath

/** @} */

#endif // __NAV_MATH_H__
