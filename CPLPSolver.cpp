#include "CPLPSolver.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#include "MathUtils.h"


CPLPSolver::CPLPSolver()
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
}

void CPLPSolver::AddConstraintLinear(float A, float B, float C)
{
	constraints.reserve(constraints.size() + 3);
	constraints.push_back(A);
	constraints.push_back(B);
	constraints.push_back(C);
	
	constraintTypes.push_back(CT_LINEAR);
}

void CPLPSolver::AddConstraintCircle(float U, float V, float R)
{
	constraints.reserve(constraints.size() + 3);
	constraints.push_back(U);
	constraints.push_back(V);
	constraints.push_back(R);
	
	constraintTypes.push_back(CT_CIRCLE);
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
	for(int i = 0; i < arr.size(); i++)
	{
		
		std::cout << arr[i];
		if(i % 3 == 2)
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
	for(int i = 0; i < order.size(); i++)
	{
		std::cout << order[i] << " ";
	}
	std::cout << "\n";
}

bool CPLPSolver::pointSatisfiesConstraint(float tx, float ty, int n, float d)
{
	if(filter.count(n) == 0)
	{
		if(constraintTypes[n] == CT_LINEAR && constraints[n * 3] * tx + constraints[n * 3 + 1] * ty > constraints[n * 3 + 2] + d) // + EPS  to the end?)
		{
			return false;
		}
		else if (constraintTypes[n] == CT_CIRCLE &&
			(tx - constraints[n * 3]) * (tx - constraints[n * 3]) + (ty - constraints[n * 3 + 1]) * (ty - constraints[n * 3 + 1]) > constraints[n * 3 + 2] * constraints[n * 3 + 2])
		{
			return false;
		}
	}
	
	return true;
}

bool CPLPSolver::pointSatisfiesConstraints(float tx, float ty, int n, float d, bool onlyCircles)
{
	for(int j = 0; j < n; j++)
	{
		if(onlyCircles && constraintTypes[order[j]] == CT_LINEAR)
		{
			continue;
		}
		if(!pointSatisfiesConstraint(tx, ty, order[j], d))
		{
			return false;
		}
	}
	return true;
}

void CPLPSolver::createRandomOrder()
{
	order.clear();
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		order.push_back(i);
	}
	std::random_shuffle(order.begin(), order.end());
}

void CPLPSolver::normalizeLinearConstraints()
{
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(constraintTypes[i] == CT_CIRCLE)
		{
			continue;
		}
		float lrec = 1.f / sqrtf(constraints[3 * i] * constraints[3 * i] + constraints[3 * i + 1] * constraints[3 * i + 1]);
		constraints[3 * i] *= lrec;
		constraints[3 * i + 1] *= lrec;
		constraints[3 * i + 2] *= lrec;
	}
}

float CPLPSolver::getPointsMaxDistance(float x, float y)
{
	float dmax = 0.f;
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(constraintTypes[i] == CT_CIRCLE)
		{
			continue;
		}
		float td = constraints[3 * i] * x + constraints[3 * i + 1] * y - constraints[3 * i + 2];
		if(td > dmax)
		{
			dmax = td;
		}
	}
	return dmax;
}

void CPLPSolver::updatePointIfBetter(float x, float y, int n, float& resX, float& resY, float& d)
{
	if(pointSatisfiesConstraints(x, y, n, 0.f, true))
	{
		float td = getPointsMaxDistance(x, y);
		if(td < d)
		{
			resX = x;
			resY = y;
			d = td;
		}
	}
}

void CPLPSolver::Solve(float& resX, float& resY)
{
	//printArray(constraints);
	
	filter.clear();
	createRandomOrder();
	
	float tx = 0.f;
	float ty = 0.f;
	float tx2 = 0.f;
	float ty2 = 0.f;
	
	resX = u;
	resY = v;
	
	//printOrder(order);
	
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(pointSatisfiesConstraint(resX, resY, order[i]))
		{
			continue;
		}
		int id = order[i];
		int pos = id * 3;
		
		if(constraintTypes[id] == CT_LINEAR)
		{
			OrthogonalProjectionOfPointOnLine(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
		}
		else
		{
			OrthogonalProjectionOfPointOnCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
		}
		filter.clear();
		filter.insert(id);
		
		float minDistSqr = 1e9;
		float tdist;
		if(pointSatisfiesConstraints(tx, ty, i))
		{
			tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
			minDistSqr = tdist;
			resX = tx;
			resY = ty;
		}
		
		
		for(int j = 0; j < i; j++)
		{
			int jd = order[j];
			int pos2 = jd * 3;
			
			filter.clear();
			filter.insert(id);
			filter.insert(jd);
			
			bool onePoint = false;
			bool hasIntersection = false;
			if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = IntersectLines(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty);
				onePoint = true;
			}
			else if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = IntersectLineCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
			}
			else if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = IntersectLineCircle(constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], constraints[pos], constraints[pos + 1], constraints[pos + 2], tx, ty, tx2, ty2);
			}
			else if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = IntersectCircleCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
				if(!hasIntersection)
				{
					feasible = false;
					return;
				}
			}
			if(hasIntersection)
			{
				if(pointSatisfiesConstraints(tx, ty, i))
				{
					tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
					if(tdist < minDistSqr)
					{
						resX = tx;
						resY = ty;
						minDistSqr = tdist;
					}
				}
				
				if(!onePoint && pointSatisfiesConstraints(tx2, ty2, i))
				{
					tdist = (tx2 - u) * (tx2 - u) + (ty2 - v) * (ty2 - v);
					if(tdist < minDistSqr)
					{
						resX = tx2;
						resY = ty2;
						minDistSqr = tdist;
					}
				}
			}
		}
		if(minDistSqr == 1e9)
		{
			SolveSafest(resX, resY);
			return;
		}
	}
	
	//TODO optimizing current solution : currently runtime should be O(n^2) where n is number of constraints,
	// but the average runtime should be O(n) if implementation is the same as in [De Berg et al. 2008]
}

void CPLPSolver::SolveSafest(float& resX, float& resY)
{
	filter.clear();
	createRandomOrder();
	
	normalizeLinearConstraints();
	
	float tx = 0.f;
	float ty = 0.f;
	float tx2 = 0.f;
	float ty2 = 0.f;
	
	float d = 0.f;
	resX = u;
	resY = v;
	
	//printOrder(order);
	
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(pointSatisfiesConstraint(resX, resY, order[i], d))
		{
			continue;
		}
		int id = order[i];
		int pos = id * 3;
		
		if(constraintTypes[id] == CT_LINEAR)
		{
			OrthogonalProjectionOfPointOnLine(constraints[pos], constraints[pos + 1], constraints[pos + 2] + d, u, v, tx, ty);
		}
		else
		{
			OrthogonalProjectionOfPointOnCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
		}
		filter.clear();
		filter.insert(id);
		
		float minDistSqr = 1e9;
		float tdist;
		if(pointSatisfiesConstraints(tx, ty, i, d))
		{
			tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
			minDistSqr = tdist;
			resX = tx;
			resY = ty;
		}
		
		
		for(int j = 0; j < i; j++)
		{
			int jd = order[j];
			int pos2 = jd * 3;
			
			filter.clear();
			filter.insert(id);
			filter.insert(jd);
			
			
			bool onePoint = false;
			bool hasIntersection = false;
			if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = IntersectLines(constraints[pos], constraints[pos + 1], constraints[pos + 2] + d, constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2] + d, tx, ty);
				onePoint = true;
			}
			else if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = IntersectLineCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2] + d, constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
			}
			else if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = IntersectLineCircle(constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2] + d, constraints[pos], constraints[pos + 1], constraints[pos + 2], tx, ty, tx2, ty2);
			}
			else if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = IntersectCircleCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
				if(!hasIntersection)
				{
					feasible = false;
					return;
				}
			}
			if(hasIntersection)
			{
				if(pointSatisfiesConstraints(tx, ty, i, d))
				{
					tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
					if(tdist < minDistSqr)
					{
						resX = tx;
						resY = ty;
						minDistSqr = tdist;
					}
				}
				
				if(!onePoint && pointSatisfiesConstraints(tx2, ty2, i, d))
				{
					tdist = (tx2 - u) * (tx2 - u) + (ty2 - v) * (ty2 - v);
					if(tdist < minDistSqr)
					{
						resX = tx2;
						resY = ty2;
						minDistSqr = tdist;
					}
				}
			}
		}
		if(minDistSqr == 1e9)
		{
			//calculate new d
			
			d = 1e9;
				
			for(int j = 0; j < i; j++)
			{
				int jd = order[j];
				int pos2 = jd * 3;
				
				filter.clear();
				filter.insert(id);
				filter.insert(jd);
				
				if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_CIRCLE)
				{
					IntersectCircleCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
					updatePointIfBetter(tx, ty, i, resX, resY, d);
					updatePointIfBetter(tx2, ty2, i, resX, resY, d);
				}
				
				//circle centers projected towards the line
				if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR)
				{
					tx = constraints[pos] - constraints[pos2] * constraints[pos + 2];
					ty = constraints[pos + 1] - constraints[pos2 + 1] * constraints[pos + 2];
					updatePointIfBetter(tx, ty, i, resX, resY, d);
				}
				if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE)
				{
					tx = constraints[pos2] - constraints[pos] * constraints[pos2 + 2];
					ty = constraints[pos2 + 1] - constraints[pos + 1] * constraints[pos2 + 2];
					updatePointIfBetter(tx, ty, i, resX, resY, d);
				}
				
				if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR)
				{
					float A, B, C;
					AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], A, B, C);
					OrthogonalProjectionOfPointOnLine(A, B, C, u, v, tx, ty);
					updatePointIfBetter(tx, ty, i, resX, resY, d);
				}
				
				for(int k = j + 1; k < i; k++)
				{
					int kd = order[k];
					int pos3 = kd * 3;
					filter.clear();
					filter.insert(id);
					filter.insert(jd);
					filter.insert(kd);
					
					float G1, H1, I1, G2, H2, I2;
					if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR && constraintTypes[kd] == CT_LINEAR)
					{
						AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], G1, H1, I1);
						AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], G2, H2, I2);
						IntersectLines(G1, H1, I1, G2, H2, I2, tx, ty);
						updatePointIfBetter(tx, ty, i, resX, resY, d);
					}
					
					if(constraintTypes[jd] == CT_CIRCLE && constraintTypes[kd] == CT_CIRCLE)
					{
						IntersectCircleCircle(constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], tx, ty, tx2, ty2);
						updatePointIfBetter(tx, ty, i, resX, resY, d);
						updatePointIfBetter(tx2, ty2, i, resX, resY, d);
					}
					
					if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR && constraintTypes[kd] == CT_CIRCLE)
					{
						AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], G1, H1, I1);
						IntersectLineCircle(G1, H1, I1, constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], tx, ty, tx2, ty2);
						updatePointIfBetter(tx, ty, i, resX, resY, d);
						updatePointIfBetter(tx2, ty2, i, resX, resY, d);
					}
					
					if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE && constraintTypes[kd] == CT_LINEAR)
					{
						AngleBisector(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], G1, H1, I1);
						IntersectLineCircle(G1, H1, I1, constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
						updatePointIfBetter(tx, ty, i, resX, resY, d);
						updatePointIfBetter(tx2, ty2, i, resX, resY, d);
					}
					
					if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR && constraintTypes[kd] == CT_LINEAR)
					{
						AngleBisector(constraints[pos3], constraints[pos3 + 1], constraints[pos3 + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], G1, H1, I1);
						IntersectLineCircle(G1, H1, I1, constraints[pos], constraints[pos + 1], constraints[pos + 2], tx, ty, tx2, ty2);
						updatePointIfBetter(tx, ty, i, resX, resY, d);
						updatePointIfBetter(tx2, ty2, i, resX, resY, d);
					}
					
					
				}
				
			}
			
			
			
		}
	}

}