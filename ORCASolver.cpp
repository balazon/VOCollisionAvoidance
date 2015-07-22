#include "ORCASolver.h"

#include <cmath>

Agent::Agent()
{
	std::fill_n(nearbyAgents, CA_MAXNEARBY, -1);
}

Agent::Agent(float x, float y, float vx, float vy, float r, float vx_pref, float vy_pref) : x{x}, y{y}, vx{vx}, vy{vy}, r{r}, vx_pref{vx_pref}, vy_pref{vy_pref}
{
}

Agent::~Agent()
{
}


ORCASolver::ORCASolver() : num{0}, T{2.f}
{
}

ORCASolver::~ORCASolver()
{
}


Agent& ORCASolver::GetAgent(int& id)
{
	if(replacingIds.count(id) > 0)
	{
		int oldid = id;
		id = replacingIds[id];
		replacingIds.erase(oldid);
	}
	return agents[id];
}

//returns id of agent
int ORCASolver::AddAgent()
{
	return num++;
}

void ORCASolver::RemoveAgent(int id)
{
	replacingIds[num - 1] = id;
	num--;
}

void ORCASolver::ClearAgents()
{
	num = 0;
}

void ORCASolver::SetParameters(float T)
{
	this->T = T;
}


//Math stuff here

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
	float c = C * C - 2.f * v * C * B - r * r * B * B;
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
	return IntersectLineCircle(u1 - u2, v1 - v2, (r2 + r1) * (r2 - r1), u1, v1, r1, x1, y1, x2, y2);
}



void ORCASolver::ComputeNewVelocities()
{
	replacingIds.clear();
	
	//TODO not implemented yet - 
}