/**-------------------------------------------------------------------------
@file	math_so3.h

@brief	SO(3) rotation and quaternion geometry

Single precision rotation group helpers: SO(3) exponential and left Jacobian,
rotation matrix re-orthonormalisation, and quaternion operations. Depends on
math_linalg.h for vector and 3x3 primitives.

Conventions: quaternion layout is [w x y z]; rotation matrices are row-major;
a quaternion built by QuatToDcmBE or a matrix from DcmFromGravity maps body
frame vectors to the earth frame (v_earth = R v_body).

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
#ifndef __MATH_SO3_H__
#define __MATH_SO3_H__

#include "math_linalg.h"

namespace So3 {

using namespace LinAlg;

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


} // namespace So3

#endif // __MATH_SO3_H__
