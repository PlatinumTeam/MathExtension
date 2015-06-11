//-----------------------------------------------------------------------------
// MathExtension: A C++ Implementation of a ton of TorqueScript math functions
// Most code by Whirligig
// Ported to C++ by Jeff/HiGuy
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Copyright(c) 2015 Whirligig231
// Copyright(c) 2015 Jeff Hutchinson
// Copyright(c) 2015 HiGuy Smith
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//-----------------------------------------------------------------------------

#ifndef MathExtension_h
#define MathExtension_h

#include "StringMath.h"

//Functions

/**
 * Projects vector u onto vector v.
 * @arg u The vector to project.
 * @arg v The vector on which to project.
 * @return The projected vector.
 */
inline Point3F VectorProj(const Point3F &u, const Point3F &v) {
	return v * (mDot(u, v) / mDot(v, v));
}

/**
 * Gets the length of the projection of vector u onto vector v.
 * @arg u The vector to project.
 * @arg v The vector on which to project.
 * @return The lenght of the projection vector.
 */
inline F32 VectorProjLen(const Point3F &u, const Point3F &v) {
	return mDot(u, v) / v.len();
}

/**
 * Rejects vector u onto vector v (component of u perpendicular to v).
 * @arg u The vector to reject.
 * @arg v The vector on which to reject.
 * @return The rejected vector.
 */
inline Point3F VectorRej(const Point3F &u, const Point3F &v) {
	return u - VectorProj(u, v);
}

/**
 * Calculates the inverse of a matrix.
 * @arg mat The matrix to inverse.
 * @return The inverted matrix.
 */
inline MatrixF MatrixInverse(MatrixF mat) {
	mat.inverse();
	return mat;
}

/**
 * Rotates one vector by an axis and angle.
 * @arg vec The vector to rotate.
 * @arg axis The axis about which to rotate the vector.
 * @arg angle The angle by which the vector is rotated.
 * @return The rotated vector.
 */
inline Point3F VectorRotate(const Point3F &vec, Point3F axis, const F32 &angle) {
	axis.normalize();
	Point3F part1 = vec * mCos(angle);
	Point3F part2 = mCross(axis, vec) * mSin(angle);
	Point3F part3 = axis * mDot(axis, vec) * (1 - mCos(angle));
	return part1 + part2 + part3;
}

/**
 * Find the angle between two vectors.
 * @arg u The first vector.
 * @arg v The second vector.
 * @return The angle between u and v.
 */
inline F32 VectorAngle(const Point3F &u, const Point3F &v) {
	return mAcos(mDot(u, v) / u.len() / v.len());
}

/**
 * Crosses one vector by another, with regard to special edge-case angles (180 degrees).
 * @arg u The first vector to cross.
 * @arg v The second vector to cross.
 * @return The cross-product of the vectors.
 */
inline Point3F VectorCrossSpecial(const Point3F &u, const Point3F &v) {
	if (VectorAngle(u, v) >= M_PI_F) {
		if (mFabs(u.x) < 0.01 && mFabs(u.y) < 0.01) {
			return mCross(u, Point3F(1, 0, 0));
		}
		return mCross(u, Point3F(0, 0, 1));
	}
	return mCross(u, v);
}

/**
 * Find the axis of rotation between two vectors.
 * @arg u The first vector.
 * @arg v The second vector.
 * @return The axis of rotation between u and v.
 */
inline Point3F VectorAxis(const Point3F &u, const Point3F &v) {
	Point3F ret = mCross(u, v);
	ret.normalize();
	return ret;
}

/**
 * Find the axis-angle rotation between two vectors.
 * @arg u The first vector.
 * @arg v The second vector.
 * @return The axis-angle rotation between u and v.
 */
inline AngAxisF VectorRot(const Point3F &u, const Point3F &v) {
	AngAxisF a;
	a.axis = VectorAxis(u, v);
	a.angle = VectorAngle(u, v);
	return a;
}

/**
 * Divide one matrix by another.
 * @arg mat1 The dividend matrix.
 * @arg mat2 The divisor matrix.
 * @return The quotient of the division mat1 / mat2.
 */
inline MatrixF MatrixDivide(const MatrixF &mat1, MatrixF mat2) {
	mat2.inverse();
	return mat1 * mat2;
}

/**
 * Interpolate between one axis-angle rotation and another.
 * @arg rot1 The first rotation.
 * @arg rot2 The second rotation.
 * @arg t The current state of interpolation (0.0 - 1.0).
 * @return The partial rotation from rot1 to rot2.
 */
inline AngAxisF RotInterpolate(const AngAxisF &rot1, const AngAxisF &rot2, const F32 &t) {
	MatrixF mat1;
	MatrixF mat2;
	rot1.setMatrix(&mat1);
	rot2.setMatrix(&mat2);

	MatrixF matSub = MatrixDivide(mat2, mat1);
	AngAxisF a(matSub);
	a.angle *= t;

	MatrixF newMat;
	a.setMatrix(&newMat);
	newMat *= mat1;

	return AngAxisF(newMat);
}

/**
 * Interpolate between one matrix and another.
 * @arg mat1 The first matrix.
 * @arg mat2 The second matrix.
 * @arg t The current state of interpolation (0.0 - 1.0).
 * @return The partial interpolation from rot1 to rot2.
 */
inline MatrixF MatInterpolate(const MatrixF &mat1, const MatrixF &mat2, const F32 &t) {
	Point3F pos1 = mat1.getPosition();
	Point3F pos2 = mat2.getPosition();

	AngAxisF rot1(mat1);
	AngAxisF rot2(mat2);

	//Interpolate from pos1 -> pos2, store into posI
	Point3F posI;
	posI.interpolate(pos1, pos2, t);
	//Interpolate from rot1 -> rot2, store into rotI
	AngAxisF rotI = RotInterpolate(rot1, rot2, t);

	MatrixF ret;
	rotI.setMatrix(&ret);
	ret.setPosition(posI);

	return ret;
}

/**
 * Calculates the factorial of a number.
 * @arg val The number whose factorial to get.
 * @return The factorial of val.
 */
inline U64 mFact(const U64 &val) {
	U32 ret = 1;
	for (U32 i = 2; i < val; i++) {
		ret *= i;
	}
	return ret;
}

/**
 * Gets the 2D point in the view from (-1,-1) to (1,1) corresponding to a point in 2D space
 * @arg transform The current camera transformation
 * @arg worldSpace The point to project, in world space
 * @arg fov The field of view
 * @arg recurse Internally used for backwards angles.
 */
Point2F getGuiSpace(const MatrixF &transform, const Point3F &worldSpace, const F32 fov, bool recurse = false);

#endif