#include "ORCASolver.h"

#include <cmath>
#include <utility>
#define EPS (0.001)

#include "CPLPSolver.h"

CPLPSolver solver;

Agent::Agent() : Agent{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
{
}

Agent::Agent(float x, float y, float vx, float vy, float r, float vx_pref, float vy_pref) : x{x}, y{y}, vx{vx}, vy{vy}, r{r}, vx_pref{vx_pref}, vy_pref{vy_pref}
{
	std::fill_n(nearbyAgents, CA_MAXNEARBY, -1);
}

Agent::~Agent()
{
}

void ORCASolver::ClearNeighbours(int i)
{
	Agent& a = agents[i];
	std::fill_n(a.nearbyAgents, CA_MAXNEARBY, -1);
}
void ORCASolver::SetAgentsNearby(int i, int j)
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

bool ORCASolver::AreAgentsNeighbours(int i, int j)
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
void ORCASolver::SetUVector(int i, int j, float ux, float uy)
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

ORCASolver::ORCASolver() : T{2.f}, num{0}
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
		return -1;
	
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
	float r = R / T;
	float Ox = ABx / T;
	float Oy = ABy / T;
	float Px, Py;
	float Qx, Qy;
	
	//bool IntersectCircleCircle(float u1, float v1, float r1, float u2, float v2, float r2, float& x1, float& y1, float& x2, float& y2)
	
	
	IntersectCircleCircle(ABx, ABy, R, Ox, Oy, sqrtf(Ox * Ox + Oy * Oy), Px, Py, Qx, Qy);
	
	if(-ABy * Px + ABx * Py > 0.f)
	{
		std::swap(Px, Qx);
		std::swap(Py, Qy);
	}
	
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
	
	//G, H points are not needed, but they are O's orthogonal projections to AP, and AQ (or the little circle's touching point)
	//they are the 'g' and 'h' in variable names below: Aog, Aoh, ..
	
	float vrelx = a.vx - b.vx;
	float vrely = a.vy - b.vy;
	
	if(Npx * vrelx + Npy * vrely > 0 || Nqx * vrelx + Nqy * vrely > 0)
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
	float Sx, Sy;
	if(Aog * vrelx + Bog * vrely < Cog || Aoh * vrelx + Boh * vrely < Coh)
	{
		if(-ABy * vrelx + ABx * vrely > 0.f)
		{
			OrthogonalProjectionOfPointOnLine(Nqx, Nqy, 0.f, vrelx, vrely, Sx, Sy);
		}
		else
		{
			OrthogonalProjectionOfPointOnLine(Npx, Npy, 0.f, vrelx, vrely, Sx, Sy);
		}
	}
	else if ((vrelx - Ox) * (vrelx - Ox) + (vrely - Oy) * (vrely - Oy) < r * r)
	{
		float x1,x2,y1,y2;
		IntersectLineCircle(Oy - vrely, vrelx - Ox, (Oy - vrely) * Ox + (vrelx - Ox) * Oy, Ox, Oy, r, x1, y1, x2, y2);
		if(Aog * x1 + Bog * y1 > Cog || Aoh * x1 + Boh * y1 > Coh)
		{
			Sx = x1;
			Sy = y1;
		}
		else
		{
			Sx = x2;
			Sy = y2;
		}
	}
	else
	{
		SetUVector(i, j, 0.f, 0.f);
		return;
	}
	
	SetUVector(i, j, Sx - vrelx, Sy - vrely);
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
				computeSmallestChangeVectors(i, j);
			}
		}
		Agent& a = agents[i];
		solver.Reset();
		solver.SetDestination(a.vx_pref, a.vy_pref);
		for(int j = 0; j < CA_MAXNEARBY; j++)
		{
			float ux = a.ux[j];
			float uy = a.uy[j];
			if(a.nearbyAgents[j] != -1 && (ux != 0.f || uy != 0.f))
			{
				float A = ux;
				float B = uy;
				//V_a + u / 2, direction is u
				float C = A * (a.vx + ux * .5f) + B * (a.vy + uy * .5f);
				if(A * a.vx + B * a.vy < C)
				{
					A = -A;
					B = -B;
					C = -C;
				}
				solver.AddConstraint(A, B, C);
			}
		}
		
		solver.Solve(a.vx_new, a.vy_new);
		//if solver fails (solver.HasSolution is false) - what to do?
	}
}