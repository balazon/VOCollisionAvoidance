#include "MathUtils.h"

#include <cmath>
#include <utility>
#define EPS (0.001)

//intersection point of two lines : Ax + By = C, Dx + Ey = F
//returns false if parallel
bool IntersectLines(float A, float B, float C, float D, float E, float F, float& resX, float& resY)
{
	float denominator = D * B - E * A;
	if(fabs(denominator) < EPS) return false;
	
	resX = (F * B - E * C) / denominator;
	resY = (C * D - A * F) / denominator;
	return true;
}


//line: Ax + By = C
//point: (tx, ty)
void OrthogonalProjectionOfPointOnLine(float A, float B, float C, float tx, float ty, float& resX, float& resY)
{
	float denominator = A * A + B * B;
	resX = (B * B * tx - A * B * ty + A * C) / denominator;
	resY = (A * A * ty - A * B * tx + B * C) / denominator;
}

bool QuadraticEquation(float a, float b, float c, float& x1, float& x2)
{
	float D;
	if(fabs(a) < EPS)
	{
		x1 = x2 = - c / b;
		return true;
	}
	if((D = b * b - 4 * a * c) < 0)
	{
		return false;
	}
	float denom_reciproc = .5f / a;
	float sqrtD = sqrtf(D);
	x1 = (- b + sqrtD) * denom_reciproc;
	x2 = (- b - sqrtD) * denom_reciproc;
	return true;
}

bool IntersectLineCircle(float A, float B, float C, float u, float v, float r, float& x1, float& y1, float& x2, float& y2)
{
	bool swapped = false;
	if(fabs(B) < EPS)
	{
		swapped = true;
		std::swap(u, v);
		std::swap(A, B);
	}
	float a = A * A + B * B;
	float b = - 2.f * C * A - 2.f * u * B * B + 2.f * v * A * B;
	float c = C * C - 2.f * v * C * B + (u * u + v * v - r * r) * B * B;
	bool solvable = QuadraticEquation(a, b, c, x1, x2);
	if(!solvable)
	{
		return false;
	}
	y1 = (C - A * x1) / B;
	y2 = (C - A * x2) / B;
	if(swapped)
	{
		std::swap(x1, y1);
		std::swap(x2, y2);
	}
	return true;
}

bool IntersectCircleCircle(float u1, float v1, float r1, float u2, float v2, float r2, float& x1, float& y1, float& x2, float& y2)
{
	return IntersectLineCircle(u1 - u2, v1 - v2, .5f * ((r2 + r1) * (r2 - r1) + (u1 + u2) * (u1 - u2) + (v1 + v2) * (v1 - v2)), u1, v1, r1, x1, y1, x2, y2);
}

void OrthogonalProjectionOfPointOnCircle(float u, float v, float r, float tx, float ty, float& resX, float& resY)
{
	float OTx = tx - u;
	float OTy = ty - v;
	float k = r / sqrtf(OTx * OTx + OTy * OTy);
	resX = u + OTx * k;
	resY = v + OTy * k;
}
