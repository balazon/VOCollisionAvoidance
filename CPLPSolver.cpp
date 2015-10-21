// Fill out your copyright notice in the Description page of Project Settings.

//#include "RVOTest.h"
#include "CPLPSolver.h"






#include <cmath>
#include <iostream>
#include <algorithm>

#include "MathUtils.h"


CPLPSolver::CPLPSolver() : debug{ false }, fixedElementsNum{0}
{
	constraints.reserve(30);
	constraintTypes.reserve(30);
	order.reserve(30);
	
	
}

CPLPSolver::~CPLPSolver()
{
}

void CPLPSolver::Reset()
{
	constraints.clear();
	feasible = true;
	constraintTypes.clear();
	fixedElementsNum = 0;
}

void CPLPSolver::AddConstraintLinear(float A, float B, float C, bool fixed)
{
	/*if (fabs(A) < EPS && fabs(B) < EPS || BMU::isnanf(A) || BMU::isnanf(B) || BMU::isnanf(C))
	{
		UE_LOG(LogRVOTest, Warning, TEXT("BAMM"));
	}*/
	float lrec = 1.f / sqrtf(A * A + B * B);
	//UE_LOG(LogRVOTest, Warning, TEXT("addcl %f %f %f lrec: %f"), A, B, C, lrec);

	constraints.reserve(constraints.size() + 3);
	constraints.push_back(A * lrec);
	constraints.push_back(B * lrec);
	constraints.push_back(C * lrec);

	constraintTypes.push_back(CT_LINEAR);

	if (fixed)
	{
		auto oldPos = constraints.end() - 3;
		auto newPos = constraints.begin() + fixedElementsNum * 3;
		std::swap_ranges(newPos, newPos + 3, oldPos);
		std::swap(constraintTypes[fixedElementsNum], *constraintTypes.rbegin());
		fixedElementsNum++;
	}

	int max = constraintTypes.size() > constraints.size() / 3 ? constraintTypes.size() : constraints.size() / 3;
	for (int i = 0; i < max; i++)
	{
		int pos = i * 3;
		float A = constraints[pos];
		float B = constraints[pos + 1];
		float C = constraints[pos + 2];

		if (fabs(A) < EPS && fabs(B) < EPS)
		{
			UE_LOG(LogRVOTest, Warning, TEXT("BAMM AddConstraintLinear has it: %f %f %f"), A, B, C);
		}
	}
}

void CPLPSolver::AddConstraintCircle(float U, float V, float R, bool fixed)
{
	for (int i = 0; i < constraints.size() / 3; i++)
	{
		if (constraintTypes[i] == CT_CIRCLE)
		{
			//when new circle is completely inside the another, the smaller one is enough
			float rsub = constraints[i * 3 + 2] - R;
			//float rsubAbs = fabsf(rsub);
			
			

			float circDistSq = (constraints[i * 3] - U) * (constraints[i * 3] - U) + (constraints[i * 3 + 1] - V) * (constraints[i * 3 + 1] - V);
			//if one circle is completely inside the other
			if (circDistSq < rsub * rsub + fabs(rsub * EPS))
			{
				// 1) new one is smaller, replaces the big
				if (rsub > 0.f)
				{
					constraints[i * 3] = U;
					constraints[i * 3 + 1] = V;
					constraints[i * 3 + 2] = R;

					//old is not fixed
					if (fixed && i >= fixedElementsNum)
					{
						auto oldPos = constraints.begin() + i * 3;
						auto newPos = constraints.begin() + (fixedElementsNum - 1) * 3;
						std::swap_ranges(newPos, newPos + 3, oldPos);
					}
				}

				// 2) otherwise we ignore the new one
				return;
			}
		}
	}
	constraints.reserve(constraints.size() + 3);
	constraints.push_back(U);
	constraints.push_back(V);
	constraints.push_back(R);

	constraintTypes.push_back(CT_CIRCLE);

	if (fixed)
	{
		auto oldPos = constraints.end() - 3;
		auto newPos = constraints.begin() + fixedElementsNum * 3;
		std::swap_ranges(newPos, newPos + 3, oldPos);
		std::swap(constraintTypes[fixedElementsNum], *constraintTypes.rbegin());
		fixedElementsNum++;
	}
}

void CPLPSolver::SetDestination(float u, float v)
{
	this->u = u;
	this->v = v;
}

bool CPLPSolver::HasSolution()
{
	return feasible;
}

//for debugging
void printArray(std::vector<float>& arr)
{
	std::cout << "tomb: ";
	for (int i = 0; i < arr.size(); i++)
	{

		std::cout << arr[i];
		if (i % 3 == 2)
			std::cout << ", ";
		else
			std::cout << " ";
	}
	std::cout << "\n";
}

//for debugging
void printOrder(std::vector<int>& order)
{
	std::cout << "order: ";
	for (int i = 0; i < order.size(); i++)
	{
		std::cout << order[i] << " ";
	}
	std::cout << "\n";
}

bool CPLPSolver::pointSatisfiesConstraint(float tx, float ty, int n, float d)
{
	if (filter.count(n) == 0)
	{
		if (constraintTypes[n] == CT_LINEAR && constraints[n * 3] * tx + constraints[n * 3 + 1] * ty + EPS > constraints[n * 3 + 2] + d ) // + EPS  to the end?)
		{
			return false;
		}
		else if (constraintTypes[n] == CT_CIRCLE &&
			(tx - constraints[n * 3]) * (tx - constraints[n * 3]) + (ty - constraints[n * 3 + 1]) * (ty - constraints[n * 3 + 1]) + EPS > constraints[n * 3 + 2] * constraints[n * 3 + 2])
		{
			return false;
		}
	}

	return true;
}

bool CPLPSolver::pointSatisfiesConstraints(float tx, float ty, int n, float d, bool onlyCircles)
{
	if (debug)
	{
		UE_LOG(LogRVOTest, Warning, TEXT("pointsatisfies: tx ty n d %f %f %d %f"), tx, ty, n, d);
	}
	for (int j = 0; j <= n; j++)
	{
		if (onlyCircles && constraintTypes[order[j]] == CT_LINEAR)
		{
			continue;
		}
		if (!pointSatisfiesConstraint(tx, ty, order[j], d))
		{
			if (debug)
			{
				UE_LOG(LogRVOTest, Warning, TEXT("pointsatisfies: false"));
			}
			return false;
		}
	}
	if (debug)
	{
		UE_LOG(LogRVOTest, Warning, TEXT("pointsatisfies: true"));
	}
	return true;
}

void CPLPSolver::createRandomOrder()
{
	order.clear();
	for (int i = 0; i < constraints.size() / 3; i++)
	{
		order.push_back(i);
	}
	std::random_shuffle(order.begin() + fixedElementsNum, order.end());
}

void CPLPSolver::normalizeLinearConstraints()
{
	for (int i = 0; i < constraints.size() / 3; i++)
	{
		if (constraintTypes[i] == CT_CIRCLE)
		{
			continue;
		}
		float lrec = 1.f / sqrtf(constraints[3 * i] * constraints[3 * i] + constraints[3 * i + 1] * constraints[3 * i + 1]);
		if (BMU::isnanf(lrec))
		{
			std::cout << "normalize BAM\n";
		}
		constraints[3 * i] *= lrec;
		constraints[3 * i + 1] *= lrec;
		constraints[3 * i + 2] *= lrec;
	}
}

float CPLPSolver::getPointsMaxDistance(float x, float y)
{
	float dmax = 0.f;
	for (int i = 0; i < constraints.size() / 3; i++)
	{
		if (constraintTypes[i] == CT_CIRCLE)
		{
			continue;
		}
		float td = constraints[3 * i] * x + constraints[3 * i + 1] * y - constraints[3 * i + 2];
		if (td > dmax)
		{
			dmax = td;
		}
	}
	return dmax;
}

void CPLPSolver::updatePointIfBetter(float x, float y, int n, float& resX, float& resY, float& d)
{
	if (debug)
	{
		UE_LOG(LogRVOTest, Warning, TEXT("updatepointif:  x y n %f %f %d"), x, y, n);
	}
	if (pointSatisfiesConstraints(x, y, n, 0.f, true))
	{
		
		float td = getPointsMaxDistance(x, y);
		if (td < d)
		{
			if (debug)
			{
				UE_LOG(LogRVOTest, Warning, TEXT("updatepointif updating, old d, new d %f %f"), d, td);
			}
			resX = x;
			resY = y;
			d = td;
			if (BMU::isnanf(resX) || BMU::isnanf(resY))
			{
				UE_LOG(LogRVOTest, Warning, TEXT("BAM in: updatepointifbetter"));
			}
			
		}

		
	}

	else if (debug)
	{
		UE_LOG(LogRVOTest, Warning, TEXT("updatepointif:  not updating"));
	}
}

void CPLPSolver::Solve(float& resX, float& resY)
{
	printArray(constraints);

	usedSafest = false;

	filter.clear();
	createRandomOrder();

	float tx = 0.f;
	float ty = 0.f;
	float tx2 = 0.f;
	float ty2 = 0.f;

	resX = u;
	resY = v;

	printOrder(order);

	if (debug)
	{
		UE_LOG(LogRVOTest, Warning, TEXT("u v : %f %f  "), u, v);

		UE_LOG(LogRVOTest, Warning, TEXT("All constraints: "));

		for (int i = 0; i < constraints.size() / 3; i++)
		{
			int id = order[i];
			int pos = id * 3;
			if (constraintTypes[id] == CT_LINEAR)
			{
				UE_LOG(LogRVOTest, Warning, TEXT("%d lin: %f %f %f "), id, constraints[pos], constraints[pos + 1], constraints[pos + 2]);
			}
			else
			{
				UE_LOG(LogRVOTest, Warning, TEXT("%d cir: %f %f %f "), id, constraints[pos], constraints[pos + 1], constraints[pos + 2]);
			}
			
		}
	}
	
	

	for (int i = 0; i < constraints.size() / 3; i++)
	{
		if (pointSatisfiesConstraint(resX, resY, order[i]))
		{
			continue;
		}
		int id = order[i];
		int pos = id * 3;

		
		

		bool first = false;
		if (constraintTypes[id] == CT_LINEAR)
		{
			BMU::OrthogonalProjectionOfPointOnLine(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
			first = true;
		}
		else
		{
			first = BMU::OrthogonalProjectionOfPointOnCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
			if (!first)
			{
				UE_LOG(LogRVOTest, Warning, TEXT("%d. constr : %f %f %f  "), id, constraints[pos], constraints[pos + 1], constraints[pos + 2]);
			}
		}
		
		filter.clear();
		filter.insert(id);

		feasible = false;
		float minDistSqr = -1.f;
		float tdist;
		
		if (first && pointSatisfiesConstraints(tx, ty, i))
		{
			feasible = true;
			tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
			minDistSqr = tdist;
			resX = tx;
			resY = ty;
			if (BMU::isnanf(resX) || BMU::isnanf(resY))
			{
				UE_LOG(LogRVOTest, Warning, TEXT("cplpsolver::solve  %f %f %f %f %f %f %f"), constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
			}
		}
		


		for (int j = 0; j < i; j++)
		{
			int jd = order[j];
			int pos2 = jd * 3;

			filter.clear();
			filter.insert(id);
			filter.insert(jd);

			bool onePoint = false;
			bool hasIntersection = false;
			if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = BMU::IntersectLines(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty);
				onePoint = true;
			}
			else if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = BMU::IntersectLineCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
			}
			else if (constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = BMU::IntersectLineCircle(constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], constraints[pos], constraints[pos + 1], constraints[pos + 2], tx, ty, tx2, ty2);
			}
			else if (constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_CIRCLE)
			{
				
				hasIntersection = BMU::IntersectCircleCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
				if (!hasIntersection)
				{
					float U1, V1, R1, U2, V2, R2;
					U1 = constraints[pos]; V1 = constraints[pos + 1]; R1 = constraints[pos + 2]; U2 = constraints[pos2]; V2 = constraints[pos2 + 1]; R2 = constraints[pos2 + 2];

					UE_LOG(LogRVOTest, Warning, TEXT("cir cir: %f %f %f %f %f %f"), U1, V1, R1, U2, V2, R2);

					BMU::debug = true;
					BMU::IntersectCircleCircle(U1, V1, R1, U2, V2, R2, tx, ty, tx2, ty2);
					BMU::debug = false;

					Reset();
					if (pos > pos2)
					{
						AddConstraintCircle(U2, V2, R2);
						AddConstraintCircle(U1, V1, R1);
					}
					else
					{
						AddConstraintCircle(U1, V1, R1);
						AddConstraintCircle(U2, V2, R2);
					}
					
					feasible = false;
					return;
				}
				
			}
			if (hasIntersection)
			{
				if (pointSatisfiesConstraints(tx, ty, i))
				{
					tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
					if (!feasible || tdist < minDistSqr)
					{
						feasible = true;
						resX = tx;
						resY = ty;
						minDistSqr = tdist;
						if (BMU::isnanf(resX) || BMU::isnanf(resY))
						{
							if (constraintTypes[id] == CT_LINEAR)
							{
								UE_LOG(LogRVOTest, Warning, TEXT("lin: "));
							}
							else
							{
								UE_LOG(LogRVOTest, Warning, TEXT("cir: "));
							}
								
							UE_LOG(LogRVOTest, Warning, TEXT("%f %f %f "), constraints[pos], constraints[pos + 1], constraints[pos + 2]);
						}
					}
				}

				if (!onePoint && pointSatisfiesConstraints(tx2, ty2, i))
				{
					tdist = (tx2 - u) * (tx2 - u) + (ty2 - v) * (ty2 - v);
					if (!feasible || tdist < minDistSqr)
					{
						feasible = true;
						resX = tx2;
						resY = ty2;
						minDistSqr = tdist;
						if (BMU::isnanf(resX) || BMU::isnanf(resY))
						{
							if (constraintTypes[jd] == CT_LINEAR)
							{
								UE_LOG(LogRVOTest, Warning, TEXT("lin: "));
							}
							else
							{
								UE_LOG(LogRVOTest, Warning, TEXT("cir: "));
							}
							UE_LOG(LogRVOTest, Warning, TEXT("%f %f %f "), constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2]);
						}
					}
				}
			}
		}
		if (!feasible)
		{
			SolveSafest(i, resX, resY);
			feasible = true;
			return;
		}
	}

	//TODO optimizing current solution : currently runtime should be O(n^2) where n is number of constraints,
	// but the average runtime should be O(n) if implementation is the same as in [De Berg et al. 2008]
}

void CPLPSolver::SolveSafest(int failIndex, float& resX, float& resY)
{
	
	usedSafest = true;

	filter.clear();
	

	//normalizeLinearConstraints();

	float tx = 0.f;
	float ty = 0.f;
	float tx2 = 0.f;
	float ty2 = 0.f;

	float d = 0.f;
	

	//printOrder(order);

	for (int i = failIndex; i < constraints.size() / 3; i++)
	{
		if (pointSatisfiesConstraint(resX, resY, order[i], d))
		{
			continue;
		}
		int id = order[i];
		int pos = id * 3;

		bool first = false;
		if (constraintTypes[id] == CT_LINEAR)
		{
			BMU::OrthogonalProjectionOfPointOnLine(constraints[pos], constraints[pos + 1], constraints[pos + 2] + d, u, v, tx, ty);
			first = true;
		}
		else
		{
			first = BMU::OrthogonalProjectionOfPointOnCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
		}
		filter.clear();
		filter.insert(id);

		
		bool solvable = false;
		float minDistSqr = -1.f;
		float tdist;
		if (first && pointSatisfiesConstraints(tx, ty, i, d))
		{
			solvable = true;
			tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
			minDistSqr = tdist;
			resX = tx;
			resY = ty;
			if (BMU::isnanf(resX) || BMU::isnanf(resY))
			{
				UE_LOG(LogRVOTest, Warning, TEXT("params: %f %f %f %f %f %f %f"), constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
			}
		}


		for (int j = 0; j < i; j++)
		{
			int jd = order[j];
			int pos2 = jd * 3;

			filter.clear();
			filter.insert(id);
			filter.insert(jd);


			bool onePoint = false;
			bool hasIntersection = false;
			if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = BMU::IntersectLines(constraints[pos], constraints[pos + 1], constraints[pos + 2] + d, constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2] + d, tx, ty);
				onePoint = true;
			}
			else if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = BMU::IntersectLineCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2] + d, constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
			}
			else if (constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = BMU::IntersectLineCircle(constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2] + d, constraints[pos], constraints[pos + 1], constraints[pos + 2], tx, ty, tx2, ty2);
			}
			else if (constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_CIRCLE)
			{
				
				hasIntersection = BMU::IntersectCircleCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
				if (!hasIntersection)
				{
					feasible = false;
					return;
				}
			}
			if (hasIntersection)
			{
				if (pointSatisfiesConstraints(tx, ty, i, d))
				{
					tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
					if (!solvable || tdist < minDistSqr)
					{
						solvable = true;
						resX = tx;
						resY = ty;
						minDistSqr = tdist;
						if (BMU::isnanf(resX) || BMU::isnanf(resY))
						{
							if (constraintTypes[id] == CT_LINEAR)
							{
								UE_LOG(LogRVOTest, Warning, TEXT("lin: "));
							}
							else
							{
								UE_LOG(LogRVOTest, Warning, TEXT("cir: "));
							}

							UE_LOG(LogRVOTest, Warning, TEXT("%f %f %f "), constraints[pos], constraints[pos + 1], constraints[pos + 2]);
						}
					}

				}

				if (!onePoint && pointSatisfiesConstraints(tx2, ty2, i, d))
				{
					tdist = (tx2 - u) * (tx2 - u) + (ty2 - v) * (ty2 - v);
					if (!solvable || tdist < minDistSqr)
					{
						solvable = true;
						resX = tx2;
						resY = ty2;
						minDistSqr = tdist;
						if (BMU::isnanf(resX) || BMU::isnanf(resY))
						{
							if (constraintTypes[jd] == CT_LINEAR)
							{
								UE_LOG(LogRVOTest, Warning, TEXT("lin: "));
							}
							else
							{
								UE_LOG(LogRVOTest, Warning, TEXT("cir: "));
							}
							UE_LOG(LogRVOTest, Warning, TEXT("%f %f %f "), constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2]);
						}
					}
				}
			}
		}

		if (!solvable)
		{
			//calculate new d

			d = 1e9;

			for (int j = 0; j < i; j++)
			{
				int jd = order[j];
				int pos2 = jd * 3;

				filter.clear();
				filter.insert(id);
				filter.insert(jd);

				if (constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_CIRCLE)
				{
					bool intersects = BMU::IntersectCircleCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
					if (intersects)
					{
						updatePointIfBetter(tx, ty, i, resX, resY, d);
						updatePointIfBetter(tx2, ty2, i, resX, resY, d);
					}
					
				}

				//circle centers projected towards the line
				//(for (u, v, r) circle and a normalized (A, B, C) line this point is: (u,v) - r * (A, B)
				//TODO investigate possible d value that would make the line touch the circle (is it enough or not)
				else if (constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR)
				{
					tx = constraints[pos] - constraints[pos2] * constraints[pos + 2];
					ty = constraints[pos + 1] - constraints[pos2 + 1] * constraints[pos + 2];
					updatePointIfBetter(tx, ty, i, resX, resY, d);
				}
				else if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE)
				{
					tx = constraints[pos2] - constraints[pos] * constraints[pos2 + 2];
					ty = constraints[pos2 + 1] - constraints[pos + 1] * constraints[pos2 + 2];
					updatePointIfBetter(tx, ty, i, resX, resY, d);
				}

				else if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR)
				{
					float A, B, C;
					bool hasBisector = BMU::AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], A, B, C);
					if (hasBisector)
					{
						BMU::OrthogonalProjectionOfPointOnLine(A, B, C, u, v, tx, ty);
						updatePointIfBetter(tx, ty, i, resX, resY, d);
					}
				}

				for (int k = j + 1; k < i; k++)
				{
					int kd = order[k];
					int pos3 = kd * 3;
					filter.clear();
					filter.insert(id);
					filter.insert(jd);
					filter.insert(kd);

					float G1, H1, I1, G2, H2, I2;
					if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR && constraintTypes[kd] == CT_LINEAR)
					{
						bool hasBisector = BMU::AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], G1, H1, I1)
							&& BMU::AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], G2, H2, I2);
						if (hasBisector)
						{
							bool intersects = BMU::IntersectLines(G1, H1, I1, G2, H2, I2, tx, ty);
							if (intersects)
							{
								updatePointIfBetter(tx, ty, i, resX, resY, d);
							}
						}
					}

					else if (constraintTypes[jd] == CT_CIRCLE && constraintTypes[kd] == CT_CIRCLE)
					{
						
						bool intersects = BMU::IntersectCircleCircle(constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], tx, ty, tx2, ty2);
						if (intersects)
						{
							updatePointIfBetter(tx, ty, i, resX, resY, d);
							updatePointIfBetter(tx2, ty2, i, resX, resY, d);
						}
						
					}

					else if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR && constraintTypes[kd] == CT_CIRCLE)
					{
						bool hasBisector = BMU::AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], G1, H1, I1);
						if (hasBisector)
						{
							bool intersects = BMU::IntersectLineCircle(G1, H1, I1, constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], tx, ty, tx2, ty2);
							if (intersects)
							{
								updatePointIfBetter(tx, ty, i, resX, resY, d);
								updatePointIfBetter(tx2, ty2, i, resX, resY, d);
							}
						}
					}

					else if (constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE && constraintTypes[kd] == CT_LINEAR)
					{
						bool hasBisector = BMU::AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], G1, H1, I1);
						if (hasBisector)
						{
							bool intersects = BMU::IntersectLineCircle(G1, H1, I1, constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
							if (intersects)
							{
								updatePointIfBetter(tx, ty, i, resX, resY, d);
								updatePointIfBetter(tx2, ty2, i, resX, resY, d);
							}
						}
					}

					else if (constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR && constraintTypes[kd] == CT_LINEAR)
					{
						bool hasBisector = BMU::AngleBisector(constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], G1, H1, I1);
						if (hasBisector)
						{
							bool intersects = BMU::IntersectLineCircle(G1, H1, I1, constraints[pos], constraints[pos + 1], constraints[pos + 2], tx, ty, tx2, ty2);
							if (intersects)
							{
								updatePointIfBetter(tx, ty, i, resX, resY, d);
								updatePointIfBetter(tx2, ty2, i, resX, resY, d);
							}
						}
					}


				}

			}



		}
	}

	usedDInSafest = d;
}