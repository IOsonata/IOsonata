/**-------------------------------------------------------------------------
@file	fusion_math.h

@brief	Shared math kernel for software imu fusion backends

Single precision vector, 3x3 matrix, SO(3) and quaternion helpers used by the
software fusion backends (EqF, MEKF, Mahony). All functions are inline and in
the FusionMath namespace so the same definitions can be included by several
translation units without a link conflict. No CMSIS-DSP dependency.

Quaternion layout is [w x y z]. Rotation matrices are row-major. A quaternion
built by QuatToDcmBE or returned by DcmFromGravity maps body frame vectors to
the earth frame (v_earth = R v_body).

@author	Hoang Nguyen Hoan
@date	Jun. 23, 2026

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
#ifndef __IMU_MATH_H__
#define __IMU_MATH_H__

#include <math.h>
#include <string.h>

/** @addtogroup IMU
  * @{
  */

namespace FusionMath {

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

// ---- SO(3) ----

// SO(3) exponential. v = angle * axis -> rotation matrix R.
inline void Rodrigues(const float v[3], float R[9])
{
	float angle = Norm3(v);
	if (angle < 1.0e-8f) {
		Mat3Eye(R);
		R[1] -= v[2]; R[2] += v[1];
		R[3] += v[2]; R[5] -= v[0];
		R[6] -= v[1]; R[7] += v[0];
		return;
	}
	float c = cosf(angle), s = sinf(angle), t = 1.0f - c;
	float ia = 1.0f / angle;
	float x = v[0] * ia, y = v[1] * ia, z = v[2] * ia;
	R[0] = c + t * x * x;     R[1] = t * x * y - s * z; R[2] = t * x * z + s * y;
	R[3] = t * y * x + s * z; R[4] = c + t * y * y;     R[5] = t * y * z - s * x;
	R[6] = t * z * x - s * y; R[7] = t * z * y + s * x; R[8] = c + t * z * z;
}

// SO(3) left Jacobian.
inline void So3LeftJ(const float v[3], float J[9])
{
	float angle = Norm3(v);
	if (angle < 1.0e-6f) {
		Mat3Eye(J);
		float h = 0.5f;
		J[1] -= h * v[2]; J[2] += h * v[1];
		J[3] += h * v[2]; J[5] -= h * v[0];
		J[6] -= h * v[1]; J[7] += h * v[0];
		return;
	}
	float s = sinf(angle), c = cosf(angle), ia = 1.0f / angle;
	float x = v[0] * ia, y = v[1] * ia, z = v[2] * ia;
	float sa = s * ia;
	float ms = 1.0f - sa;
	float mc = (1.0f - c) * ia;
	J[0] = sa + ms * x * x;     J[1] = ms * x * y - mc * z; J[2] = ms * x * z + mc * y;
	J[3] = ms * y * x + mc * z; J[4] = sa + ms * y * y;     J[5] = ms * y * z - mc * x;
	J[6] = ms * z * x - mc * y; J[7] = ms * z * y + mc * x; J[8] = sa + ms * z * z;
}

// Gram-Schmidt re-orthonormalise a row-major rotation matrix.
inline void Reortho(float A[9])
{
	float r0[3] = { A[0], A[1], A[2] };
	float r1[3] = { A[3], A[4], A[5] };
	float r2[3];
	Normalize3(r0);
	float d = Dot3(r1, r0);
	r1[0] -= d * r0[0]; r1[1] -= d * r0[1]; r1[2] -= d * r0[2];
	Normalize3(r1);
	Cross3(r0, r1, r2);
	A[0] = r0[0]; A[1] = r0[1]; A[2] = r0[2];
	A[3] = r1[0]; A[4] = r1[1]; A[5] = r1[2];
	A[6] = r2[0]; A[7] = r2[1]; A[8] = r2[2];
}

// ---- quaternion, [w x y z] ----

// Rotation matrix to quaternion, Shepperd's method, positive w.
inline void MatToQuat(const float R[9], float q[4])
{
	float tr = R[0] + R[4] + R[8];
	if (tr > 0.0f) {
		float s = 0.5f / sqrtf(tr + 1.0f);
		q[0] = 0.25f / s;
		q[1] = (R[7] - R[5]) * s;
		q[2] = (R[2] - R[6]) * s;
		q[3] = (R[3] - R[1]) * s;
	} else if (R[0] > R[4] && R[0] > R[8]) {
		float s = 2.0f * sqrtf(1.0f + R[0] - R[4] - R[8]);
		q[0] = (R[7] - R[5]) / s;
		q[1] = 0.25f * s;
		q[2] = (R[1] + R[3]) / s;
		q[3] = (R[2] + R[6]) / s;
	} else if (R[4] > R[8]) {
		float s = 2.0f * sqrtf(1.0f + R[4] - R[0] - R[8]);
		q[0] = (R[2] - R[6]) / s;
		q[1] = (R[1] + R[3]) / s;
		q[2] = 0.25f * s;
		q[3] = (R[5] + R[7]) / s;
	} else {
		float s = 2.0f * sqrtf(1.0f + R[8] - R[0] - R[4]);
		q[0] = (R[3] - R[1]) / s;
		q[1] = (R[2] + R[6]) / s;
		q[2] = (R[5] + R[7]) / s;
		q[3] = 0.25f * s;
	}
	if (q[0] < 0.0f) {
		q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];
	}
}

inline void QuatNormalize(float q[4])
{
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n > 1.0e-12f) {
		float inv = 1.0f / n;
		q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
	}
}

// Hamilton product out = a (x) b.
inline void QuatMul(const float a[4], const float b[4], float out[4])
{
	float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
	out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

// Quaternion from a rotation vector (angle * axis).
inline void QuatFromRotVec(const float v[3], float q[4])
{
	float angle = Norm3(v);
	if (angle < 1.0e-8f) {
		q[0] = 1.0f;
		q[1] = 0.5f * v[0]; q[2] = 0.5f * v[1]; q[3] = 0.5f * v[2];
		QuatNormalize(q);
		return;
	}
	float half = 0.5f * angle;
	float s = sinf(half) / angle;
	q[0] = cosf(half);
	q[1] = v[0] * s; q[2] = v[1] * s; q[3] = v[2] * s;
}

// Body to earth rotation matrix from quaternion, row-major.
inline void QuatToDcmBE(const float q[4], float R[9])
{
	float w = q[0], x = q[1], y = q[2], z = q[3];
	R[0] = 1.0f - 2.0f * (y * y + z * z);
	R[1] = 2.0f * (x * y - w * z);
	R[2] = 2.0f * (x * z + w * y);
	R[3] = 2.0f * (x * y + w * z);
	R[4] = 1.0f - 2.0f * (x * x + z * z);
	R[5] = 2.0f * (y * z - w * x);
	R[6] = 2.0f * (x * z - w * y);
	R[7] = 2.0f * (y * z + w * x);
	R[8] = 1.0f - 2.0f * (x * x + y * y);
}

// ---- 6x6 covariance block access, row-major P[36] ----

inline void PGet(const float P[36], int bi, int bj, float B[9])
{
	int r = bi * 3, c = bj * 3;
	for (int i = 0; i < 3; i++) {
		const float *row = P + (r + i) * 6 + c;
		B[i * 3 + 0] = row[0];
		B[i * 3 + 1] = row[1];
		B[i * 3 + 2] = row[2];
	}
}

inline void PSet(float P[36], int bi, int bj, const float B[9])
{
	int r = bi * 3, c = bj * 3;
	for (int i = 0; i < 3; i++) {
		float *row = P + (r + i) * 6 + c;
		row[0] = B[i * 3 + 0];
		row[1] = B[i * 3 + 1];
		row[2] = B[i * 3 + 2];
	}
}

// ---- attitude initialisation ----

// Build a body to earth rotation matrix from an averaged gravity vector. The
// third row is set to the unit gravity direction; the remaining heading degree
// of freedom is fixed by an arbitrary reference, since 6-axis cannot observe
// it. Sets identity if the input is degenerate.
inline void DcmFromGravity(const float accAvg[3], float R[9])
{
	float gn = Norm3(accAvg);
	if (gn < 1.0e-6f) {
		Mat3Eye(R);
		return;
	}
	float inv = 1.0f / gn;
	float b1[3] = { accAvg[0] * inv, accAvg[1] * inv, accAvg[2] * inv };

	float ref[3] = { 1.0f, 0.0f, 0.0f };
	if (fabsf(Dot3(b1, ref)) > 0.9f) {
		ref[0] = 0.0f; ref[1] = 1.0f; ref[2] = 0.0f;
	}
	float b2[3];
	Cross3(b1, ref, b2);
	float b2n = Norm3(b2);
	if (b2n < 1.0e-6f) {
		Mat3Eye(R);
		return;
	}
	inv = 1.0f / b2n;
	b2[0] *= inv; b2[1] *= inv; b2[2] *= inv;
	float b3[3];
	Cross3(b1, b2, b3);
	R[0] = -b3[0]; R[1] = -b3[1]; R[2] = -b3[2];
	R[3] =  b2[0]; R[4] =  b2[1]; R[5] =  b2[2];
	R[6] =  b1[0]; R[7] =  b1[1]; R[8] =  b1[2];
}

} // namespace FusionMath

/** @} */

#endif // __IMU_MATH_H__
