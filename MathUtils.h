// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

//#define UE_BUILD



#ifndef UE_BUILD

class LogRVOTest;

#define VeryVerbose (4)
#define Warning (5)
#define LogLevel (5)
#define TEXT(x) x
#define UE_LOG(x, y, ...) if(y >= LogLevel) printf(__VA_ARGS__)
#include <cstdio>

#endif



#include <cmath>
#include <utility>




//bmu for bala math utils
namespace BMU
{
	extern bool debug;
	//intersection point of two lines : Ax + By = C, Dx + Ey = F
	//returns false if parallel
	bool IntersectLines(float A, float B, float C, float D, float E, float F, float& resX, float& resY);

	//line: Ax + By = C
	//point: (tx, ty)
	void OrthogonalProjectionOfPointOnLine(float A, float B, float C, float tx, float ty, float& resX, float& resY);

	bool QuadraticEquation(float a, float b, float c, float& x1, float& x2);

	bool IntersectLineCircle(float A, float B, float C, float u, float v, float r, float& x1, float& y1, float& x2, float& y2);

	bool IntersectCircleCircle(float u1, float v1, float r1, float u2, float v2, float r2, float& x1, float& y1, float& x2, float& y2);

	//if the point is in the center, then no solution
	bool OrthogonalProjectionOfPointOnCircle(float u, float v, float r, float tx, float ty, float& resX, float& resY);

	//2 lines' bisector: Ax + By = C, Dx + Ey = F, result: Gx + Hy = I
	// (A,B) and (D,E) must be normalized when passing it in
	// the resulting line's normal is (A,B) - (D,E)
	// this should work for parallel lines as well
	bool AngleBisector(float A, float B, float C, float D, float E, float F, float& G, float& H, float& I);





	bool isnanf(float k);



#define EPS (0.0001)


	bool inline isnanf(float k)
	{
		return std::isnan(k) || !std::isfinite(k) || !std::isnormal(k) && k != 0.f || fabs(k) > 999999.f;
	}


	bool inline IntersectLines(float A, float B, float C, float D, float E, float F, float& resX, float& resY)
	{
		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("il params: %f %f %f %f %f %f"), A, B, C, D, E, F);

		float denominator = D * B - E * A;
		if (fabs(denominator) < EPS) return false;

		resX = (F * B - E * C) / denominator;
		resY = (C * D - A * F) / denominator;

		if (isnanf(resX) || isnanf(resY))
		{
			UE_LOG(LogRVOTest, Warning, TEXT("il params: %f %f %f %f %f %f"), A, B, C, D, E, F);
		}
		return true;
	}



	void inline OrthogonalProjectionOfPointOnLine(float A, float B, float C, float tx, float ty, float& resX, float& resY)
	{
		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("oppl params: %f %f %f %f %f"), A, B, C, tx, ty);

		float denominator = A * A + B * B;
		resX = (B * B * tx - A * B * ty + A * C) / denominator;
		resY = (A * A * ty - A * B * tx + B * C) / denominator;
		if (isnanf(resX) || isnanf(resY))
		{
			UE_LOG(LogRVOTest, Warning, TEXT("oppl params: %f %f %f %f %f"), A, B, C, tx, ty);
		}
	}

	bool inline QuadraticEquation(float a, float b, float c, float& x1, float& x2)
	{
		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("qe params: %f %f %f"), a, b, c);


		float D;
		if (fabs(a) < EPS)
		{
			if (fabs(b) < EPS)
			{
				return false;
			}
			x1 = x2 = -c / b;
			return true;
		}


		if ((D = b * b - 4 * a * c) < 0)
		{
			return false;
		}
		float denom_reciproc = .5f / a;
		float sqrtD = sqrtf(D);
		x1 = (-b + sqrtD) * denom_reciproc;
		x2 = (-b - sqrtD) * denom_reciproc;

		if (isnanf(x1) || isnanf(x2))
		{
			UE_LOG(LogRVOTest, Warning, TEXT("qe params: %f %f %f"), a, b, c);
		}

		return true;
	}

	bool inline IntersectLineCircle(float A, float B, float C, float u, float v, float r, float& x1, float& y1, float& x2, float& y2)
	{
		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("ilc params: %f %f %f %f %f %f"), A, B, C, u, v, r);


		if (fabs(A) < EPS && fabs(B) < EPS)
		{
			UE_LOG(LogRVOTest, VeryVerbose, TEXT("ilc params: %f %f %f %f %f %f"), A, B, C, u, v, r);
			return false;
		}
		if (fabs(B) < fabs(A))
		{
			return IntersectLineCircle(B, A, C, v, u, r, y1, x1, y2, x2);
		}
		float a = A * A + B * B;
		float b = -2.f * C * A - 2.f * u * B * B + 2.f * v * A * B;
		float c = C * C - 2.f * v * C * B + (u * u + v * v - r * r) * B * B;
		bool solvable = QuadraticEquation(a, b, c, x1, x2);
		if (!solvable)
		{
			return false;
		}
		y1 = (C - A * x1) / B;
		y2 = (C - A * x2) / B;

		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("ilc x1 y1 x2 y2 %f %f %f %f"), x1, y1, x2, y2);



		OrthogonalProjectionOfPointOnCircle(u, v, r, x1, y1, x1, y1);
		OrthogonalProjectionOfPointOnCircle(u, v, r, x2, y2, x2, y2);

		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("ilc x1 y1 x2 y2 %f %f %f %f"), x1, y1, x2, y2);

		if (isnanf(x1) || isnanf(x2) || isnanf(y1) || isnanf(y2))
		{
			UE_LOG(LogRVOTest, Warning, TEXT("ilc params: %f %f %f %f %f %f"), A, B, C, u, v, r);
		}

		return true;
	}

	bool inline IntersectCircleCircle(float u1, float v1, float r1, float u2, float v2, float r2, float& x1, float& y1, float& x2, float& y2)
	{
		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("icc params: %f %f %f %f %f %f"), u1, v1, r1, u2, v2, r2);

		float A = u1 - u2;
		float B = v1 - v2;
		float C = .5f * ((r2 + r1) * (r2 - r1) + (u1 + u2) * (u1 - u2) + (v1 + v2) * (v1 - v2));
		float uvdist = sqrtf(A * A + B * B);
		if (fabs(r1 - r2) > uvdist || uvdist > r1 + r2)
		{
			return false;
		}
		float lrec = 1.f / uvdist;
		A *= lrec;
		B *= lrec;
		C *= lrec;
		if (fabs(A) < EPS && fabs(B) < EPS)
		{
			return false;
		}
		return IntersectLineCircle(A, B, C, u1, v1, r1, x1, y1, x2, y2);
	}

	bool inline OrthogonalProjectionOfPointOnCircle(float u, float v, float r, float tx, float ty, float& resX, float& resY)
	{
		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("oppc params: %f %f %f %f %f"), u, v, r, tx, ty);

		float OTx = tx - u;
		float OTy = ty - v;
		if (fabs(OTx) < EPS && fabs(OTy) < EPS)
		{
			return false;
		}
		float k = r / sqrtf(OTx * OTx + OTy * OTy);
		resX = u + OTx * k;
		resY = v + OTy * k;

		if (isnanf(resX) || isnanf(resY))
		{
			UE_LOG(LogRVOTest, Warning, TEXT("oppc params: %f %f %f %f %f"), u, v, r, tx, ty);
		}

		return true;
	}

	// (A,B) and (D,E) must be normalized
	bool inline AngleBisector(float A, float B, float C, float D, float E, float F, float& G, float& H, float& I)
	{
		if (debug)
			UE_LOG(LogRVOTest, Warning, TEXT("anglebis: %f %f %f %f %f %f"), A, B, C, D, E, F);

		G = A - D;
		H = B - E;
		I = C - F;



		if (fabs(G) < EPS && fabs(H) < EPS)
		{
			UE_LOG(LogRVOTest, VeryVerbose, TEXT("anglebis: %f %f %f %f %f %f"), A, B, C, D, E, F);

			return false;
		}

		float lrec = 1.f / sqrtf(G * G + H * H);
		G *= lrec;
		H *= lrec;
		I *= lrec;

		return true;
	}

}