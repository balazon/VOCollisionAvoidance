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

void ClearNeighbours(int i)
{
	Agent& a = agents[i];
	std::fill_n(nearbyAgents, CA_MAXNEARBY, -1);
}
void SetAgentsNearby(int i, int j)
{
	bool aset = false;
	bool bset = false;
	Agent& a = agents[i];
	Agent& b = agents[j];
	for(int k = 0; k < CA_MAXNEARBY; k++)
	{
		if(a.nearbyAgents[k] == -1 && !aset)
		{
			a.nearbyAgents[k] = j;
			aset = true;
		}
		if(b.nearbyAgents[k] == -1 && !bset)
		{
			b.nearbyAgents[k] = i;
			bset = true;
		}
	}
}

bool AreAgentsNeighbours(int i, int j)
{
	Agent& a = agents[i];
	Agent& b = agents[j];
	for(int k = 0; k < CA_MAXNEARBY; k++)
	{
		if(a.nearbyAgents[k] == j && b.nearbyAgents[k] == i)
		{
			return true;
		}
	}
	return false;
}

//for i agent (ux, uy), for j agent (-ux, -uy)
void SetUVector(int i, int j, float ux, float uy)
{
	bool aset = false;
	bool bset = false;
	Agent& a = agents[i];
	Agent& b = agents[j];
	for(int k = 0; k < CA_MAXNEARBY; k++)
	{
		if(aset && bset)
		{
			return;
		}
		if(a.nearbyAgents[k] == j)
		{
			a.ux[k] = ux;
			a.uy[k] = uy;
			aset = true;
		}
		if(b.nearbyAgents[k] == i)
		{
			b.ux[k] = -ux;
			b.uy[k] = -uy;
			bset = true;
		}
	}
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
	if(num == CA_MAXAGENTS)
		return;
	
	std::fill_n(agents[num].nearbyAgents, CA_MAXNEARBY, -1);
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

//line: Ax + By = C
//point: (tx, ty)
void OrthogonalProjectionOfPointOnLine(float A, float B, float C, float tx, float ty, float& resX, float& resY)
{
	float denominator = A * A + B * B;
	resX = (B * B * tx - A * B * ty + A * C) / denominator;
	resY = (A * A * ty - A * B * tx + B * C) / denominator;
}

void ORCASolver::computeSmallestChangeVectors(int i, int j)
{
	Agent& a = agents[i];
	Agent& b = agents[j];
	float ABx = b.x - a.x;
	float ABy = b.y - a.y;
	float R = a.r + b.r;
	
	float Ox = ABx / T;
	float Oy = ABy / T;
	float Px, Py;
	float Qx, Qy;
	
	IntersectCircleCircle(ABx, ABy, R, Ox, Oy, R / T, Px, Py, Qx, Qy);
	
	//float constraints[3 * 4];
	float Npx = - Py;
	float Npy = Px;
	if(Npx * Ox + Npy * Oy > 0)
	{
		Npx = -Npx;
		Npy = -Npy;
	}
	float Nqx = - Qy;
	float Nqy = Qx;
	if(Nqx * Ox + Nqy * Oy > 0)
	{
		Nqx = -Nqx;
		Nqy = -Nqy;
	}
	/*actually this G and H points are not needed
	 * float Gx, Gy;
	OrthogonalProjectionOfPointOnLine(Npx, Npy, 0.f, Ox, Oy, Gx, Gy);
	float Hx, Hy;
	OrthogonalProjectionOfPointOnLine(Nqx, Nqy, 0.f, Ox, Oy, Hx, Hy);*/
	
	if(Npx * a.vx + Npy * a.vy > 0 || Nqx * a.vx + Nqy * a.vy > 0)
	{
		SetUVector(i, j, 0.f, 0.f);
		return;
	}
	
	
	//Inside OG, or OH constraint is the area that contains B
	float Aog = Px;
	float Bog = Py;
	float Cog = Aog * Ox + Bog * Oy;
	if(Aog * ABx + Bog * ABy > Cog)
	{
		Aog = - Aog;
		Bog = - Bog;
		Cog = - Cog;
	}
	float Aoh = Qx;
	float Boh = Qy;
	float Coh = Aoh * Ox + Boh * Oy;
	if(Aoh * ABx + Boh * ABy > Coh)
	{
		Aoh = - Aoh;
		Boh = - Boh;
		Coh = - Coh;
	}
	
}

void ORCASolver::ComputeNewVelocities()
{
	replacingIds.clear();
	
	//TODO not implemented yet - 
	for(int i = 0; i < num; i++)
	{
		for(int j = i + 1; j < num; j++)
		{
			if(AreAgentsNeighbours(i, j))
			{
				computeSmallestChangeVectors(int i, int j);
			}
		}
	}
}