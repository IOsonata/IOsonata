/**-------------------------------------------------------------------------
@file	math_linalg.h

@brief	Generic single precision linear algebra

Pure linear algebra with no domain meaning: 3-vector and fixed 3x3 row-major
operations, dynamic dim x dim and m x n row-major operations, and a 3x3 block
accessor for a larger row-major matrix. All functions are inline and in the
LinAlg namespace so the same definitions can be included by several
translation units without a link conflict. No CMSIS-DSP dependency.

Rotation, quaternion and any orientation specific math live in math_so3.h.

@author	Hoang Nguyen Hoan

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
#ifndef __MATH_LINALG_H__
#define __MATH_LINALG_H__

#include <math.h>
#include <string.h>

namespace LinAlg {

// ---- 3 vector ----

inline float Dot3(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline float Norm3(const float v[3])
{
	return sqrtf(Dot3(v, v));
}

inline void Cross3(const float a[3], const float b[3], float out[3])
{
	float x = a[1] * b[2] - a[2] * b[1];
	float y = a[2] * b[0] - a[0] * b[2];
	float z = a[0] * b[1] - a[1] * b[0];
	out[0] = x; out[1] = y; out[2] = z;
}

inline void Normalize3(float v[3])
{
	float n = Norm3(v);
	if (n > 1.0e-10f) {
		float inv = 1.0f / n;
		v[0] *= inv; v[1] *= inv; v[2] *= inv;
	}
}

// ---- 3x3 matrix, row-major ----

inline void Mat3Eye(float M[9])
{
	M[0] = 1; M[1] = 0; M[2] = 0;
	M[3] = 0; M[4] = 1; M[5] = 0;
	M[6] = 0; M[7] = 0; M[8] = 1;
}

inline void Mat3Copy(float dst[9], const float src[9])
{
	memcpy(dst, src, 9 * sizeof(float));
}

// C = A * B
inline void Mat3Mul(const float A[9], const float B[9], float C[9])
{
	float t[9];
	t[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	t[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	t[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
	t[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	t[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	t[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
	t[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	t[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	t[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
	memcpy(C, t, sizeof(t));
}

// C = A * B^T
inline void Mat3MulBT(const float A[9], const float B[9], float C[9])
{
	float t[9];
	t[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
	t[1] = A[0] * B[3] + A[1] * B[4] + A[2] * B[5];
	t[2] = A[0] * B[6] + A[1] * B[7] + A[2] * B[8];
	t[3] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
	t[4] = A[3] * B[3] + A[4] * B[4] + A[5] * B[5];
	t[5] = A[3] * B[6] + A[4] * B[7] + A[5] * B[8];
	t[6] = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
	t[7] = A[6] * B[3] + A[7] * B[4] + A[8] * B[5];
	t[8] = A[6] * B[6] + A[7] * B[7] + A[8] * B[8];
	memcpy(C, t, sizeof(t));
}

inline void Mat3Transpose(const float A[9], float out[9])
{
	float t[9];
	t[0] = A[0]; t[1] = A[3]; t[2] = A[6];
	t[3] = A[1]; t[4] = A[4]; t[5] = A[7];
	t[6] = A[2]; t[7] = A[5]; t[8] = A[8];
	memcpy(out, t, sizeof(t));
}

// out = M * v
inline void Mat3Vec(const float M[9], const float v[3], float out[3])
{
	float x = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
	float y = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
	float z = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
	out[0] = x; out[1] = y; out[2] = z;
}

// out = M^T * v
inline void Mat3TVec(const float M[9], const float v[3], float out[3])
{
	float x = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
	float y = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
	float z = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
	out[0] = x; out[1] = y; out[2] = z;
}

// Skew symmetric matrix of v: Skew(v) u = v x u.
inline void Skew3(const float v[3], float M[9])
{
	M[0] = 0;     M[1] = -v[2]; M[2] = v[1];
	M[3] = v[2];  M[4] = 0;     M[5] = -v[0];
	M[6] = -v[1]; M[7] = v[0];  M[8] = 0;
}

// Square of the skew of a unit direction: skew(d)^2 = d d^T - I.
inline void Skew3Sq(const float d[3], float M[9])
{
	M[0] = d[0] * d[0] - 1.0f; M[1] = d[0] * d[1];        M[2] = d[0] * d[2];
	M[3] = d[1] * d[0];        M[4] = d[1] * d[1] - 1.0f; M[5] = d[1] * d[2];
	M[6] = d[2] * d[0];        M[7] = d[2] * d[1];        M[8] = d[2] * d[2] - 1.0f;
}

inline void Mat3Sym(float M[9])
{
	float a;
	a = 0.5f * (M[1] + M[3]); M[1] = M[3] = a;
	a = 0.5f * (M[2] + M[6]); M[2] = M[6] = a;
	a = 0.5f * (M[5] + M[7]); M[5] = M[7] = a;
}

// 3x3 inverse with a scale-aware determinant guard. Returns false if singular.
inline bool Mat3Inv(const float M[9], float out[9])
{
	float det = M[0] * (M[4] * M[8] - M[5] * M[7])
		  - M[1] * (M[3] * M[8] - M[5] * M[6])
		  + M[2] * (M[3] * M[7] - M[4] * M[6]);
	float dmax = fabsf(M[0]);
	if (fabsf(M[4]) > dmax) dmax = fabsf(M[4]);
	if (fabsf(M[8]) > dmax) dmax = fabsf(M[8]);
	float thr = 1.0e-8f * dmax * dmax * dmax;
	if (thr < 1.0e-20f) thr = 1.0e-20f;
	if (fabsf(det) < thr) {
		return false;
	}
	float id = 1.0f / det;
	out[0] = (M[4] * M[8] - M[5] * M[7]) * id;
	out[1] = (M[2] * M[7] - M[1] * M[8]) * id;
	out[2] = (M[1] * M[5] - M[2] * M[4]) * id;
	out[3] = (M[5] * M[6] - M[3] * M[8]) * id;
	out[4] = (M[0] * M[8] - M[2] * M[6]) * id;
	out[5] = (M[2] * M[3] - M[0] * M[5]) * id;
	out[6] = (M[3] * M[7] - M[4] * M[6]) * id;
	out[7] = (M[1] * M[6] - M[0] * M[7]) * id;
	out[8] = (M[0] * M[4] - M[1] * M[3]) * id;
	return true;
}


// ---- dynamic dim x dim and m x n, row-major ----


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


// ---- 3x3 block of a row-major dim x dim matrix ----

// Read the 3x3 block at block row bi, block col bj into B.
inline void Mat3BlockGet(const float *M, int dim, int bi, int bj, float B[9])
{
	int r = bi * 3, c = bj * 3;
	for (int i = 0; i < 3; i++) {
		const float *row = M + (r + i) * dim + c;
		B[i * 3 + 0] = row[0];
		B[i * 3 + 1] = row[1];
		B[i * 3 + 2] = row[2];
	}
}

// Write the 3x3 block B into block row bi, block col bj.
inline void Mat3BlockSet(float *M, int dim, int bi, int bj, const float B[9])
{
	int r = bi * 3, c = bj * 3;
	for (int i = 0; i < 3; i++) {
		float *row = M + (r + i) * dim + c;
		row[0] = B[i * 3 + 0];
		row[1] = B[i * 3 + 1];
		row[2] = B[i * 3 + 2];
	}
}

} // namespace LinAlg

#endif // __MATH_LINALG_H__
