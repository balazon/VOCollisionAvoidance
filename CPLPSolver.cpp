#include "CPLPSolver.h"

#include <cmath>
#include <iostream>



#define EPS 0.001

CPLPSolver::CPLPSolver()
{
	constraints.reserve(30);
}

CPLPSolver::~CPLPSolver()
{
}

void CPLPSolver::Reset()
{
	constraints.clear();
	feasible = true;
}

void CPLPSolver::AddConstraint(float A, float B, float C)
{
	constraints.reserve(constraints.size() + 3);
	constraints.push_back(A);
	constraints.push_back(B);
	constraints.push_back(C);
}

void CPLPSolver::SetDestination(float u, float v)
{
	this->u = u;
	this->v = v;
}

bool CPLPSolver::HasSolution()
{
	//? how
	return feasible;
}
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

void processOneDim(float& minOfUpper, float& maxOfLower, float k, float q)
{
	
}

bool CPLPSolver::pointSatisfiesConstraints(float tx, float ty, const std::unordered_set<int>& filterIndexes)
{
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(filterIndexes.count(i) == 0 && constraints[i * 3] * tx + constraints[i * 3 + 1] * ty > constraints[i * 3 + 2]) // + EPS  to the end?
		{
			return false;
		}
	}
	return true;
}


//intersection point of two lines : Ax + By = C, Dx + Ey = F
//returns false if parallel
bool intersectLines(float A, float B, float C, float D, float E, float F, float& resX, float& resY)
{
	float denominator = D * B - E * A;
	if(fabs(denominator) < EPS) return false;
	
	resX = (F * B - E * C) / denominator;
	resY = (C * D - A * F) / denominator;
	return true;
}

//line: Ax + By = C
//point: (tx, ty)
void orthogonalProjectionOfPointOnLine(float A, float B, float C, float tx, float ty, float& resX, float& resY)
{
	float denominator = A * A + B * B;
	resX = (B * B * tx - A * B * ty + A * C) / denominator;
	resY = (A * A * ty - A * B * tx + B * C) / denominator;
}

void CPLPSolver::Solve(float& resX, float& resY)
{
	printArray(constraints);
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		float coeff = fabs(constraints[i * 3]);
		if (coeff < EPS)
		{
			constraints[i * 3] = 0.f;
			continue;
		}
		constraints[i * 3] = constraints[i * 3] > 0 ? 1.f : -1.f;
		constraints[i * 3 + 1] = constraints[i * 3 + 1] / coeff;
		constraints[i * 3 + 2] = constraints[i * 3 + 2] / coeff;
	}
	
	printArray(constraints);
	
	float minOfUpper = 1e9;
	float maxOfLower = 1e9;
	//maxOfLower can calculated as min, and then multiplied by (-1) if needed,
	// but multiplication is ignored now and value is treated as the negative maxOfLower value
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(constraints[i * 3] == 0.f)
		{
			//processOneDim(minOfUpper, maxOfLower, constraints[i * 3 + 1], constraints[i * 3 + 2]);
			float coeff = fabs(constraints[i * 3 + 1]);
			if(coeff < EPS)
			{
				constraints[i * 3 + 1] = 0.f;
				if(constraints[i * 3 + 2] < 0)
				{
					feasible = false;
					return;
				}
				continue;
			}
			if(constraints[i * 3 + 1] > 0 && minOfUpper > constraints[i * 3 + 2] / coeff)
			{
				minOfUpper = constraints[i * 3 + 2] / coeff;
				if(minOfUpper + maxOfLower < 0)
				{
					feasible = false;
					return;
				}
			}
			if(constraints[i * 3 + 1] < 0 && maxOfLower > constraints[i * 3 + 2] / coeff)
			{
				maxOfLower = constraints[i * 3 + 2] / coeff;
				if(minOfUpper + maxOfLower < 0)
				{
					feasible = false;
					return;
				}
			}
			continue;
		}
		for(int j = i + 1; j < constraints.size() / 3; j++)
		{
			if(constraints[j * 3] == 0.f)
			{
				continue;
			}
			if(constraints[i * 3] != constraints[j * 3])
			{
				//k * y < q
				float k = constraints[i * 3 + 1] + constraints[j * 3 + 1];
				float q = constraints[i * 3 + 2] + constraints[j * 3 + 2];
				float coeff = fabs(k);
				
				
				if(coeff < EPS)
				{
					
					if(q < 0)
					{
						feasible = false;
						return;
					}
					continue;
				}
				if(k > 0 && minOfUpper > q / coeff)
				{
					minOfUpper = q / coeff;
					if(minOfUpper + maxOfLower < 0)
					{
						feasible = false;
						return;
					}
				}
				if(k < 0 && maxOfLower > q / coeff)
				{
					maxOfLower = q / coeff;
					if(minOfUpper + maxOfLower < 0)
					{
						feasible = false;
						return;
					}
				}
				
			}
			
		}
	}
	
	//if program reached this point, it means problem is feasible
	
	float tx = u;
	float ty = v;
	
	if(pointSatisfiesConstraints(tx, ty))
	{
		//lucky - destination is inside constraints
		resX = u;
		resY = v;
		return;
	}
	
	float minDistSqr = 1e9;
	
	//a not too elegant solution: searching all intersection points, and orthogonal projection points on the constraint lines, filtering them and finding the closest one
	//constraint intersection points
	std::unordered_set<int> filter(2);
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		for(int j = i + 1; j < constraints.size() / 3; j++)
		{
			filter.clear();
			filter.insert(i);
			filter.insert(j);
			float tdist;
			if(intersectLines(constraints[i * 3], constraints[i * 3 + 1], constraints[i * 3 + 2], constraints[j * 3], constraints[j * 3 + 1], constraints[j * 3 + 2], tx, ty)
			&& pointSatisfiesConstraints(tx, ty, filter) && ((tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v)) < minDistSqr))
			{
				resX = tx;
				resY = ty;
				minDistSqr = tdist;
			}
		}
	}
	
	//orthogonal projection points
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		filter.clear();
		filter.insert(i);
		float tdist;
		orthogonalProjectionOfPointOnLine(constraints[i * 3], constraints[i * 3 + 1], constraints[i * 3 + 2], u, v, tx, ty);
		if(pointSatisfiesConstraints(tx, ty, filter) && ((tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v)) < minDistSqr))
		{
			resX = tx;
			resY = ty;
			minDistSqr = tdist;
		}
		
	}
	
	//TODO using a quadratic library, or optimizing current solution : currently runtime should be O(n^2) where n is number of constraints
}

