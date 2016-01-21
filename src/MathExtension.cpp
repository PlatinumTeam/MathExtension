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


#include <PluginLoader/PluginInterface.h>
#include <TorqueLib/TGE.h>
#include <TorqueLib/math/mMath.h>
#include <sstream>

#include "MathExtension.h"

PLUGINCALLBACK void preEngineInit(PluginInterface *plugin)
{
}

PLUGINCALLBACK void postEngineInit(PluginInterface *plugin)
{
}

PLUGINCALLBACK void engineShutdown(PluginInterface *plugin)
{
}
	
/**
 * Gets the 2D point in the view from (-1,-1) to (1,1) corresponding to a point in 2D space
 * @arg transform The current camera transformation
 * @arg worldSpace The point to project, in world space
 * @arg fov The field of view
 * @return The 2D position for the point.
 */
ConsoleFunction(getGuiSpace, const char*, 4, 4, "getGuiSpace(MatrixF transform, Point3F worldSpace, F32 fov)") {
	MatrixF mat = StringMath::scan<MatrixF>(argv[1]);
	Point3F worldSpace = StringMath::scan<Point3F>(argv[2]);
	F32 fov = StringMath::scan<F32>(argv[3]);
	Point2F gui = getGuiSpace(mat, worldSpace, fov);
	return StringMath::print(gui);
}

/**
 * Projects vector u onto vector v.
 * @arg u The vector to project.
 * @arg v The vector on which to project.
 * @return The projected vector.
 */
ConsoleFunction(VectorProj, const char*, 3, 3, "VectorProj(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);
	Point3F vec = VectorProj(u, v);
	return StringMath::print(vec);
}

/**
 * Gets the length of the projection of vector u onto vector v.
 * @arg u The vector to project.
 * @arg v The vector on which to project.
 * @return The lenght of the projection vector.
 */
ConsoleFunction(VectorProjLen, F32, 3, 3, "VectorProjLen(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);
	return VectorProjLen(u, v);
}

/**
 * Rejects vector u onto vector v (component of u perpendicular to v).
 * @arg u The vector to reject.
 * @arg v The vector on which to reject.
 * @return The rejected vector.
 */
ConsoleFunction(VectorRej, const char*, 3, 3, "VectorRej(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);
	Point3F vec = VectorRej(u, v);
	return StringMath::print(vec);
}

/**
 * Calculates the inverse of a matrix.
 * @arg mat The matrix to inverse.
 * @return The inverted matrix.
 */
ConsoleFunction(MatrixInverse, const char *, 2, 2, "MatrixInverse(MatrixF mat)") {
	MatrixF mat = StringMath::scan<MatrixF>(argv[1]);
	MatrixF ret = MatrixInverse(mat);
	return StringMath::print(ret);
}

/**
 * Rotates one vector by an axis and angle.
 * @arg vec The vector to rotate.
 * @arg axis The axis about which to rotate the vector.
 * @arg angle The angle by which the vector is rotated.
 * @return The rotated vector.
 */
ConsoleFunction(VectorRotate, const char *, 4, 4, "VectorRotate(Point3F vec, Point3F axis, F32 angle)") {
	Point3F vec = StringMath::scan<Point3F>(argv[1]);
	Point3F axis = StringMath::scan<Point3F>(argv[2]);
	F32 angle = StringMath::scan<F32>(argv[3]);
	Point3F ret = VectorRotate(vec, axis, angle);
	return StringMath::print(ret);
}

/**
 * Crosses one vector by another, with regard to special edge-case angles (180 degrees).
 * @arg u The first vector to cross.
 * @arg v The second vector to cross.
 * @return The cross-product of the vectors.
 */
ConsoleFunction(VectorCrossSpecial, const char *, 3, 3, "VectorCrossSpecial(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);
	Point3F ret = VectorCrossSpecial(u, v);
	return StringMath::print(ret);
}

/**
 * Find the angle between two vectors.
 * @arg u The first vector.
 * @arg v The second vector.
 * @return The angle between u and v.
 */
ConsoleFunction(VectorAngle, F32 , 3, 3, "VectorAngle(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);
	F32 ret = VectorAngle(u, v);
	return ret;
}

/**
 * Find the axis of rotation between two vectors.
 * @arg u The first vector.
 * @arg v The second vector.
 * @return The axis of rotation between u and v.
 */
ConsoleFunction(VectorAxis, const char *, 3, 3, "VectorAxis(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);
	Point3F ret = VectorAxis(u, v);
	return StringMath::print(ret);
}

/**
 * Find the axis-angle rotation between two vectors.
 * @arg u The first vector.
 * @arg v The second vector.
 * @return The axis-angle rotation between u and v.
 */
ConsoleFunction(VectorRot, const char *, 3, 3, "VectorRot(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);
	AngAxisF ret = VectorRot(u, v);
	return StringMath::print(ret);
}

/**
 * Divide one matrix by another.
 * @arg mat1 The dividend matrix.
 * @arg mat2 The divisor matrix.
 * @return The quotient of the division mat1 / mat2.
 */
ConsoleFunction(MatrixDivide, const char *, 3, 3, "MatrixDivide(MatrixF mat1, MatrixF mat2)") {
	MatrixF mat1 = StringMath::scan<MatrixF>(argv[1]);
	MatrixF mat2 = StringMath::scan<MatrixF>(argv[2]);
	MatrixF ret = MatrixDivide(mat1, mat2);
	return StringMath::print(ret);
}

/**
 * Interpolate between one axis-angle rotation and another.
 * @arg rot1 The first rotation.
 * @arg rot2 The second rotation.
 * @arg t The current state of interpolation (0.0 - 1.0).
 * @return The partial rotation from rot1 to rot2.
 */
ConsoleFunction(RotInterpolate, const char *, 4, 4, "RotInterpolate(AngAxisF rot1, AngAxisF rot2, F32 t)") {
	AngAxisF rot1 = StringMath::scan<AngAxisF>(argv[1]);
	AngAxisF rot2 = StringMath::scan<AngAxisF>(argv[2]);
	F32 t = StringMath::scan<F32>(argv[3]);
	AngAxisF ret = RotInterpolate(rot1, rot2, t);
	return StringMath::print(ret);
}
/**
 * Interpolate between one matrix and another.
 * @arg mat1 The first matrix.
 * @arg mat2 The second matrix.
 * @arg t The current state of interpolation (0.0 - 1.0).
 * @return The partial interpolation from rot1 to rot2.
 */
ConsoleFunction(MatInterpolate, const char *, 4, 4, "MatInterpolate(MatrixF mat1, MatrixF mat2, F32 t)") {
	MatrixF mat1 = StringMath::scan<MatrixF>(argv[1]);
	MatrixF mat2 = StringMath::scan<MatrixF>(argv[2]);
	F32 t = StringMath::scan<F32>(argv[3]);
	MatrixF ret = MatInterpolate(mat1, mat2, t);
	return StringMath::print(ret);
}

/**
 * Remove Torque's nasty scientific notation from a number. This just casts it to a string.
 * @arg val The number to process.
 * @return A string containing that number, without scientific notation.
 */
ConsoleFunction(removeScientificNotation, const char *, 2, 2, "removeScientificNotation(F32 val)") {
	F64 val = StringMath::scan<F64>(argv[1]);
	return StringMath::print(val);
}

/**
 * Calculates the factorial of a number.
 * @arg val The number whose factorial to get.
 * @return The factorial of val.
 */
ConsoleFunction(mFact, const char *, 2, 2, "mFact(U32 val)") {
	U64 val = StringMath::scan<U64>(argv[1]);
	U64 ret = mFact(val);
	return StringMath::print(ret);
}

/**
 * Constrain a number within the bounds of a minimum and maximum.
 * @arg n The number to constrain.
 * @arg min The minimum possible value for n.
 * @arg max The maximum possible value for n.
 * @return The constrained value for n from min to max.
 */
ConsoleFunction(mClamp, F32, 4, 4, "mClamp(F32 n, F32 min, F32 max)") {
	F32 n = StringMath::scan<F32>(argv[1]);
	F32 min = StringMath::scan<F32>(argv[2]);
	F32 max = StringMath::scan<F32>(argv[3]);
	F32 ret = mClampF(n, min, max);
	return ret;
}

/**
* Checks to see if a point is inside of a box.
* @arg point the point to test if it rests within the box.
* @arg box the box defined by a min and max.
* @arg checkBoundsToo test if the point can rest on the bounding box as well.
* @return true if the point is within the box, false otherwise.
*/
ConsoleFunction(isPointInsideBox, bool, 3, 4, "isPointInsideBox(%point, %box [, %checkBoundsToo]);") {
	Point3F point;
	Point3F min;
	Point3F max;
	sscanf(argv[1], "%f %f %f", &point.x, &point.y, &point.z);
	sscanf(argv[2], "%f %f %f %f %f %f", &min.x, &min.y, &min.z, &max.x, &max.y, &max.z);

	// do not check the bounds, only inside
	if (argc == 3)
		return (point.x > min.x && point.x < max.x && point.y > min.y && point.y < max.y && point.z > min.z && point.z < max.z);
	return (point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y && point.z >= min.z && point.z <= max.z);
}
//scales x by sx, y by sy, z by sz, instead of scaling x y z by s
ConsoleFunction(VectorScale2, const char *, 3, 3, "VectorScale2(Point3F u, Point3F v)") {
	Point3F u = StringMath::scan<Point3F>(argv[1]);
	Point3F v = StringMath::scan<Point3F>(argv[2]);

	Point3F ret = VectorScale2(u, v);
	return StringMath::print(ret);
}

//returns scalar distance based on input
ConsoleFunction(getThatDistance, F32, 4, 4, "getThatDistance(Point3F posOld, Point3F posNew, bool xyd)") {
	Point3F posold = StringMath::scan<Point3F>(argv[1]);
	Point3F posnew = StringMath::scan<Point3F>(argv[2]);
	bool xyd = atoi(argv[3]) != 0;

	Point3F dist = posold - posnew;
	if (xyd) {
		return Point2F(dist.x, dist.y).len();
	} else {
		return dist.len();
	}
}

ConsoleFunction(rotAAtoQ, const char *, 2, 2, "rotAAtoQ(AngAxisF aa)") {
	AngAxisF aa = StringMath::scan<AngAxisF>(argv[1]);
	return StringMath::print(rotAAtoQ(aa));
}

ConsoleFunction(rotQadd, const char *, 3, 3, "rotQadd(QuatF q1, QuatF q2)") {
	QuatF q1 = StringMath::scan<QuatF>(argv[1]);
	QuatF q2 = StringMath::scan<QuatF>(argv[2]);
	return StringMath::print(rotQadd(q1, q2));
}

ConsoleFunction(rotQmultiply, const char *, 3, 3, "rotQmultiply(QuatF q1, QuatF q2)") {
	QuatF q1 = StringMath::scan<QuatF>(argv[1]);
	QuatF q2 = StringMath::scan<QuatF>(argv[2]);
	return StringMath::print(rotQmultiply(q1, q2));
}

ConsoleFunction(rotQnormalize, const char *, 2, 2, "rotQnormalize(QuatF q)") {
	QuatF q = StringMath::scan<QuatF>(argv[1]);
	return StringMath::print(rotQnormalize(q));
}

ConsoleFunction(rotQinvert, const char *, 2, 2, "rotQinvert(QuatF q)") {
	QuatF q = StringMath::scan<QuatF>(argv[1]);
	return StringMath::print(rotQinvert(q));
}

ConsoleFunction(rotQtoVector, const char *, 3, 3, "rotQtoVector(QuatF q, VectorF v)") {
	QuatF q = StringMath::scan<QuatF>(argv[1]);
	VectorF v = StringMath::scan<VectorF>(argv[2]);
	return StringMath::print(rotQtoVector(q, v));
}

ConsoleFunction(rotQtoAA, const char *, 2, 2, "rotQtoAA(QuatF q)") {
	QuatF q = StringMath::scan<QuatF>(argv[1]);
	return StringMath::print(rotQtoAA(q));
}

ConsoleFunction(rotToVector, const char *, 3, 3, "rotToVector(AngAxisF aa, VectorF v)") {
	AngAxisF aa = StringMath::scan<AngAxisF>(argv[1]);
	VectorF v = StringMath::scan<VectorF>(argv[2]);
	return StringMath::print(rotToVector(aa, v));
}

ConsoleFunction(rotEtoAA, const char *, 3, 3, "rotEtoAA(EulerF euler, bool radians)") {
	EulerF euler = StringMath::scan<EulerF>(argv[1]);
	bool radians = atoi(argv[2]) != 0;
	return StringMath::print(rotEtoAA(euler, radians));
}

ConsoleFunction(rotVectorToAA, const char *, 2, 2, "rotVectorToAA(VectorF v)") {
	VectorF v = StringMath::scan<VectorF>(argv[1]);
	return StringMath::print(rotVectorToAA(v));
}

ConsoleFunction(isOffScreen, bool, 2, 2, "isOffScreen(Point2F point)") {
	Point2F point = StringMath::scan<Point2F>(argv[1]);
	return isOffScreen(point);
}

/**
 * Get the bezier curve coefficient for a point
 * @param n The order of the curve
 * @param i The point's index on the curve
 * @param u The current distance on the curve (normalized)
 * @return The factor by which this point's influence should be scaled
 */
ConsoleFunction(mBez, F32, 4, 4, "mBez(U32 n, U32 i, F32 u)") {
	U32 n = StringMath::scan<U32>(argv[1]);
	U32 i = StringMath::scan<U32>(argv[2]);
	F32 u = StringMath::scan<F32>(argv[3]);
	return mBez(n, i, u);
}

/**
 * Bezier curve between points
 * @param u          The current distance on the curve (normalized)
 * @param v1 v2, ... A tab-separated list of vectors
 * @return The point on the curve at the given distance
 */
ConsoleFunction(VectorBezier, const char *, 3, 3, "VectorBezier(F32 u, VectorF v1 TAB [VectorF v2 TAB [...]])") {
	F32 u = StringMath::scan<F32>(argv[1]);
	std::vector<Point3F> points;

	//Read all the points from argv[2], they should be separated by tabs
	std::stringstream ss(argv[2]);
	char line[64];
	while (ss.getline(line, 64, '\t')) {
		points.push_back(StringMath::scan<Point3F>(line));
	}

	Point3F final = VectorBezier(u, points);
	return StringMath::print(final);
}

/**
 * Bezier curve derivative between points
 * @param u          The current distance on the curve (normalized)
 * @param v1 v2, ... A tab-separated list of vectors
 * @return The derivative of the curve at the given distance
 */
ConsoleFunction(VectorBezierDeriv, const char *, 3, 3, "VectorBezierDeriv(F32 u, VectorF v1 TAB [VectorF v2 [TAB ...]])") {
	F32 u = StringMath::scan<F32>(argv[1]);
	std::vector<Point3F> points;

	//Read all the points from argv[2], they should be separated by tabs
	std::stringstream ss(argv[2]);
	char line[64];
	while (ss.getline(line, 64, '\t')) {
		points.push_back(StringMath::scan<Point3F>(line));
	}
	
	Point3F final = VectorBezierDeriv(u, points);
	return StringMath::print(final);
}

/**
 * Bezier curve between rotations
 * @param u          The current distance on the curve (normalized)
 * @param a1, a2 ... A tab-separated list of axis-angle rotations for the curve
 * @return The rotation along the curve at the given distance
 */
ConsoleFunction(RotBezier, const char *, 3, 3, "RotBezier(F32 u, AngAxisF a1 TAB [AngAxisF a2 [TAB ...]])") {
	F32 u = StringMath::scan<F32>(argv[1]);
	std::vector<AngAxisF> points;

	//Read all the rotations from argv[2], they should be separated by tabs
	std::stringstream ss(argv[2]);
	char line[64];
	while (ss.getline(line, 64, '\t')) {
		points.push_back(StringMath::scan<AngAxisF>(line));
	}

	AngAxisF final = RotBezier(u, points);
	return StringMath::print(final);
}
