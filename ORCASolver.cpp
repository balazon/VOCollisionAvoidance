// Fill out your copyright notice in the Description page of Project Settings.




#include "ORCASolver.h"

#include "MathUtils.h"
#include <cmath>
#include <utility>


#include "CPLPSolver.h"

//#define EPS (0.001)

CPLPSolver solver;

Agent::Agent() : Agent{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f }
{
}

Agent::Agent(float x, float y, float vx, float vy, float r, float vx_pref, float vy_pref, float maxVelocityMagnitude, float maxAccMagnitude)
	: x{ x }, y{ y }, vx{ vx }, vy{ vy }, r{ r }, vx_pref{ vx_pref }, vy_pref{ vy_pref }, maxVelocityMagnitude{ maxVelocityMagnitude }, maxAccMagnitude{ maxAccMagnitude }, nearbyCount{ 0 }
{
	
}

Agent::~Agent()
{
}

void ORCASolver::ClearNeighbours(int i)
{
	Agent& a = agents[i];
	a.nearbyCount = 0;
}
void ORCASolver::SetAgentsNearby(int i, int j)
{
	
	Agent& a = agents[i];
	if (a.nearbyCount >= CA_MAXNEARBY)
	{
		return;
	}
	a.nearbyAgents[a.nearbyCount] = j;
	a.nearbyCount++;
}

bool ORCASolver::AreAgentsNeighbours(int i, int j)
{
	UE_LOG(LogRVOTest, VeryVerbose, TEXT("agentsneighbours %d %d"), i, j);

	Agent& a = agents[i];
	Agent& b = agents[j];
	int aNearby = a.nearbyCount;
	int bNearby = b.nearbyCount;
	int maxN = aNearby < bNearby ? bNearby : aNearby;
	for (int k = 0; k < maxN; k++)
	{
		if (k < aNearby && a.nearbyAgents[k] == j || k < bNearby && b.nearbyAgents[k] == i)
		{
			UE_LOG(LogRVOTest, VeryVerbose, TEXT("agentsneighbours true"));
			return true;
		}
	}
	UE_LOG(LogRVOTest, VeryVerbose, TEXT("agentsneighbours false"));
	return false;
}

//for i agent (ux, uy), for j agent (-ux, -uy)
void ORCASolver::SetUVector(int i, int j, float ux, float uy, bool reversed)
{
	bool aset = false;
	bool bset = false;
	Agent& a = agents[i];
	Agent& b = agents[j];
	int aNearby = a.nearbyCount;
	int bNearby = b.nearbyCount;
	int maxN = aNearby < bNearby ? bNearby : aNearby;
	for (int k = 0; k < maxN; k++)
	{
		if (aset && bset)
		{
			return;
		}
		if (k < aNearby && a.nearbyAgents[k] == j)
		{
			a.ux[k] = ux;
			a.uy[k] = uy;
			a.uConstraintReversed[k] = reversed;
			aset = true;
		}
		if (k < bNearby && b.nearbyAgents[k] == i)
		{
			b.ux[k] = -ux;
			b.uy[k] = -uy;
			b.uConstraintReversed[k] = reversed;
			bset = true;
		}
	}
}

ORCASolver::ORCASolver() : T{ CA_TAU }, num{ 0 }
{
}

ORCASolver::~ORCASolver()
{
}


Agent& ORCASolver::GetAgent(int id)
{
	return agents[id];
}

//returns id of agent
int ORCASolver::AddAgent()
{
	if (num == CA_MAXAGENTS)
		return -1;
	
	agents[num].nearbyCount = 0;
	return num++;
}

int ORCASolver::RemoveAgent(int id)
{
	if (id >= num || id < 0)
	{
		return -1;
	}
	
	num--;
	if (id == num)
	{
		return -1;
	}
	return num;
}

void ORCASolver::ClearAgents()
{
	num = 0;
}

void ORCASolver::SetParameters(float T)
{
	this->T = T;
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

	//should the constraint area be reversed (the other half plane is allowed)
	bool reversed = false;

	//bool IntersectCircleCircle(float u1, float v1, float r1, float u2, float v2, float r2, float& x1, float& y1, float& x2, float& y2)


	BMU::IntersectCircleCircle(ABx, ABy, R, Ox, Oy, sqrtf(Ox * Ox + Oy * Oy), Px, Py, Qx, Qy);

	if (BMU::isnanf(Px) || BMU::isnanf(Py) || BMU::isnanf(Qx) || BMU::isnanf(Qy))
	{

		BMU::IntersectCircleCircle(ABx, ABy, R, Ox, Oy, sqrtf(Ox * Ox + Oy * Oy), Px, Py, Qx, Qy);
	}
	UE_LOG(LogRVOTest, VeryVerbose, TEXT("computeSmallest, P( %f , %f )  Q( %f,  %f )"), Px, Py, Qx, Qy);

	

	if (-ABy * Px + ABx * Py > 0.f)
	{
		std::swap(Px, Qx);
		std::swap(Py, Qy);
	}

	
	float Npx = -Py;
	float Npy = Px;
	if (Npx * Ox + Npy * Oy > 0)
	{
		Npx = -Npx;
		Npy = -Npy;
	}
	float Nqx = -Qy;
	float Nqy = Qx;
	if (Nqx * Ox + Nqy * Oy > 0)
	{
		Nqx = -Nqx;
		Nqy = -Nqy;
	}

	float Nplrec = 1.f / sqrtf(Npx * Npx + Npy * Npy);
	float Nqlrec = 1.f / sqrtf(Nqx * Nqx + Nqy * Nqy);

	Npx *= Nplrec;
	Npy *= Nplrec;
	Nqx *= Nqlrec;
	Nqy *= Nqlrec;
	

	//G, H points are not needed, but they are O's orthogonal projections to AP, and AQ (or the little circle's touching point)
	//they are the 'g' and 'h' in variable names below: Aog, Aoh, ..

	float vrelx = a.vx - b.vx;
	float vrely = a.vy - b.vy;

	//outside QAP angle area
	
	if (Npx * vrelx + Npy * vrely > 0 || Nqx * vrelx + Nqy * vrely > 0)
	{
		/*SetUVector(i, j, 0.f, 0.f);
		return;*/
		/*if (Npx * vrelx + Npy * vrely > a.maxAccMagnitude + b.maxAccMagnitude || Nqx * vrelx + Nqy * vrely > a.maxAccMagnitude + b.maxAccMagnitude )
		{
			SetUVector(i, j, 0.f, 0.f);
			return;
		}
		reversed = true;*/
		reversed = true;
		
	}


	//Inside OG, or OH constraint is the area that contains B
	float Aog = -Px;
	float Bog = -Py;
	float Cog = Aog * Ox + Bog * Oy;
	
	float Aoh = -Qx;
	float Boh = -Qy;
	float Coh = Aoh * Ox + Boh * Oy;
	

	float Sx, Sy;
	if (Aog * vrelx + Bog * vrely < Cog || Aoh * vrelx + Boh * vrely < Coh)
	{
		if (-ABy * vrelx + ABx * vrely > 0.f)
		{
			BMU::OrthogonalProjectionOfPointOnLine(Nqx, Nqy, 0.f, vrelx, vrely, Sx, Sy);
		}
		else
		{
			BMU::OrthogonalProjectionOfPointOnLine(Npx, Npy, 0.f, vrelx, vrely, Sx, Sy);
		}
	}
	else if ((vrelx - Ox) * (vrelx - Ox) + (vrely - Oy) * (vrely - Oy) < r * r)
	{
		BMU::OrthogonalProjectionOfPointOnCircle(Ox, Oy, r, vrelx, vrely, Sx, Sy);
	}
	else
	{
		/*SetUVector(i, j, 0.f, 0.f);
		return;*/
		/*BMU::OrthogonalProjectionOfPointOnCircle(Ox, Oy, r, vrelx, vrely, Sx, Sy);
		if ((vrelx - Sx) * (vrelx - Sx) + (vrely - Sy) * (vrely - Sy) > (a.maxAccMagnitude + b.maxAccMagnitude) * (a.maxAccMagnitude + b.maxAccMagnitude))
		{
			SetUVector(i, j, 0.f, 0.f);
			return;
		}
		reversed = true;*/
		BMU::OrthogonalProjectionOfPointOnCircle(Ox, Oy, r, vrelx, vrely, Sx, Sy);
		reversed = true;
	}

	SetUVector(i, j, Sx - vrelx, Sy - vrely, reversed);
}

void ORCASolver::ComputeNewVelocities()
{

	for (int i = 0; i < num; i++)
	{
		for (int j = i + 1; j < num; j++)
		{
			if (AreAgentsNeighbours(i, j))
			{
				computeSmallestChangeVectors(i, j);
			}
		}
		Agent& a = agents[i];
		if (a.vx_pref * a.vx_pref + a.vy_pref * a.vy_pref < EPS)
		{
			a.vx_new = 0.f;
			a.vy_new = 0.f;
			continue;
		}
		solver.Reset();
		UE_LOG(LogRVOTest, VeryVerbose, TEXT("vxpref %f %f"), a.vx_pref, a.vy_pref);
		solver.SetDestination(a.vx_pref, a.vy_pref);
		for (int j = 0; j < a.nearbyCount; j++)
		{
			float ux = a.ux[j];
			float uy = a.uy[j];
			bool reversed = a.uConstraintReversed[j];
			//a.nearbyAgents[j] == i shouldn't happen but just to be safe
			if ((fabs(ux) > EPS || fabs(uy) > EPS) && a.nearbyAgents[j] != i)
			{
				float A = reversed ? ux : -ux;
				float B = reversed ? uy : -uy;
				//point: V_a + u / 2, direction is -u, or u when reversed
				
				float C = A * (a.vx + ux * .5f) + B * (a.vy + uy * .5f);
				if (reversed)
				{
					C = A * (a.vx + ux) + B * (a.vy + uy);
				}

				solver.AddConstraintLinear(A, B, C);
				UE_LOG(LogRVOTest, VeryVerbose, TEXT("i, j: %d %d"), i, j);

				UE_LOG(LogRVOTest, VeryVerbose, TEXT("lin constr ABC: %f %f %f"), A, B, C);
				if (BMU::isnanf(A) || BMU::isnanf(B) || BMU::isnanf(C) || fabs(A) < EPS && fabs(B) < EPS)
				{
					UE_LOG(LogRVOTest, VeryVerbose, TEXT("lin constr ABC: %f %f %f"), A, B, C);
					computeSmallestChangeVectors(i, a.nearbyAgents[j]);

				}
			}
		}

		
		solver.AddConstraintCircle(a.vx, a.vy, a.maxAccMagnitude, true);
		solver.AddConstraintCircle(0.f, 0.f, a.maxVelocityMagnitude, true);
		
		

		solver.Solve(a.vx_new, a.vy_new);

		if (a.vx_new * a.vx_new + a.vy_new * a.vy_new > a.maxVelocityMagnitude * (a.maxVelocityMagnitude + 2 * EPS))
		{
			a.vx_new *= 0.99f;
			a.vy_new *= 0.99f;
			
			if (a.vx_new * a.vx_new + a.vy_new * a.vy_new > a.maxVelocityMagnitude * (a.maxVelocityMagnitude + 2 * EPS))
			{
				UE_LOG(LogRVOTest, Warning, TEXT("a.v: %f %f, a.vnew : %f %f, a.vmax: %f"), a.vx, a.vy, a.vx_new, a.vy_new, a.maxVelocityMagnitude);
				BMU::debug = true;
				solver.Solve(a.vx_new, a.vy_new);
				BMU::debug = false;
			}
			
		}

		float axnew = a.vx_new - a.vx;
		float aynew = a.vy_new - a.vy;
		if (axnew * axnew + aynew * aynew > a.maxAccMagnitude)
		{
			axnew *= 0.99f;
			aynew *= 0.99f;
			a.vx_new = a.vx + axnew;
			a.vy_new = a.vy + aynew;

			if ((a.vx_new - a.vx) * (a.vx_new - a.vx) + (a.vy_new - a.vy) * (a.vy_new - a.vy) > a.maxAccMagnitude * (a.maxAccMagnitude + 2 * EPS))
			{
				UE_LOG(LogRVOTest, Warning, TEXT("a.v %f %f, a.vnew %f %f, a.maxacc : %f"), a.vx, a.vy, a.vx_new, a.vy_new, a.maxAccMagnitude);
				BMU::debug = true;
				solver.Solve(a.vx_new, a.vy_new);
				BMU::debug = false;
			}
			
		}

		if (BMU::isnanf(a.vx_new) || BMU::isnanf(a.vy_new))
		{
			UE_LOG(LogRVOTest, Warning, TEXT("BAM\n"));
			solver.debug = true;
			solver.Solve(a.vx_new, a.vy_new);
			solver.debug = false;
			if (BMU::isnanf(a.vx_new) || BMU::isnanf(a.vy_new))
			{
				UE_LOG(LogRVOTest, VeryVerbose, TEXT("BAM confirm\n"));
			}
		}
	}
}

void ORCASolver::SetDebugging(bool on)
{
	solver.debug = on;
	BMU::debug = on;
}