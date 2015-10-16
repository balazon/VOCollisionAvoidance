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



void ORCASolver::SetORCAConstraint(Agent& a, int j, float A, float B, float C)
{
	for(int i = 0; i < a.nearbyCount; i++)
	{
		if(a.nearbyAgents[i] == j)
		{
			a.ORCAA[i] = A;
			a.ORCAB[i] = B;
			a.ORCAC[i] = C;
			break;
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



void ORCASolver::computeORCAConstraints(int i, int j)
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

	//v_opt is outside the VO
	bool outside = false;
	

	BMU::IntersectCircleCircle(ABx, ABy, R, 0, 0, sqrtf(ABx * ABx + ABy * ABy - R * R), Px, Py, Qx, Qy);
	
	UE_LOG(LogRVOTest, VeryVerbose, TEXT("computeSmallest, P( %f , %f )  Q( %f,  %f )"), Px, Py, Qx, Qy);

	

	if (-ABy * Px + ABx * Py > 0.f)
	{
		std::swap(Px, Qx);
		std::swap(Py, Qy);
	}

	float Npx = Py;
	float Npy = -Px;
	
	float Nqx = -Qy;
	float Nqy = Qx;

	float Nplrec = 1.f / sqrtf(Npx * Npx + Npy * Npy);
	float Nqlrec = 1.f / sqrtf(Nqx * Nqx + Nqy * Nqy);

	Npx *= Nplrec;
	Npy *= Nplrec;
	Nqx *= Nqlrec;
	Nqy *= Nqlrec;
	

	//G, H points are not needed, but they are O's orthogonal projections to AP, and AQ (or the little circle's touching point)
	//they are the 'g' and 'h' in variable names below: Aog, Aoh, ..
	
	//v^{opt}_A - v^{opt}_B which is vrel when v^{opt}_X = v_X for all X
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
		outside = true;
		
	}


	//Inside OG, or OH constraint is the area that contains B
	float Aog = -Px;
	float Bog = -Py;
	float Cog = Aog * Ox + Bog * Oy;
	
	float Aoh = -Qx;
	float Boh = -Qy;
	float Coh = Aoh * Ox + Boh * Oy;
	

	float Sx, Sy, Nx, Ny;
	if (Aog * vrelx + Bog * vrely < Cog || Aoh * vrelx + Boh * vrely < Coh)
	{
		if (-ABy * vrelx + ABx * vrely > 0.f)
		{
			BMU::OrthogonalProjectionOfPointOnLine(Nqx, Nqy, 0.f, vrelx, vrely, Sx, Sy);
			Nx = Nqx;
			Ny = Nqy;
		}
		else
		{
			BMU::OrthogonalProjectionOfPointOnLine(Npx, Npy, 0.f, vrelx, vrely, Sx, Sy);
			Nx = Npx;
			Ny = Npy;
		}
	}
	else
	{
		bool orth = BMU::OrthogonalProjectionOfPointOnCircle(Ox, Oy, r, vrelx, vrely, Sx, Sy);
		if(!orth)
		{
			orth = BMU::OrthogonalProjectionOfPointOnCircle(Ox, Oy, r, Ox + 2.f * Npx * EPS, Oy + 2.f * Npy * EPS, Sx, Sy);
		}
		Nx = Sx - Ox;
		Ny = Sy - Oy;
		outside = outside || ((vrelx - Ox) * (vrelx - Ox) + (vrely - Oy) * (vrely - Oy) > r * r);
	}
	float Nlrec = 1.f / sqrtf(Nx * Nx + Ny * Ny);
	Nx *= Nlrec;
	Ny *= Nlrec;
	
	float ux = Sx - vrelx;
	float uy = Sy - vrely;
	
	if(!outside)
	{
		ux *= .5f;
		uy *= .5f;
	}
	
	float A1, B1, C1, A2, B2, C2;
	A1 = -Nx; B1 = -Ny; C1 = -Nx * (a.vx + ux) - Ny * (a.vy + uy);
	A2 = Nx; B2 = Ny; C2 = Nx * (b.vx - ux) + Ny * (b.vy - uy);
	
	UE_LOG(LogRVOTest, VeryVerbose, TEXT("ij: %d %d, \n  %.2f, %.2f, %.2f \n  %.2f, %.2f, %.2f\n"), i, j, A1, B1, C1, A2, B2, C2);
	/*UE_LOG(LogRVOTest, Warning, TEXT("u: %.2f %.2f\n"), ux, uy);
	UE_LOG(LogRVOTest, Warning, TEXT("a.v: %.2f %.2f\n"), a.vx, a.vy);
	UE_LOG(LogRVOTest, Warning, TEXT("b.v: %.2f %.2f\n"), b.vx, b.vy);
	UE_LOG(LogRVOTest, Warning, TEXT("a.v + u %.2f %.2f\n"), a.vx + ux, a.vy + uy);
	UE_LOG(LogRVOTest, Warning, TEXT("b.v - u %.2f %.2f\n\n"), b.vx - ux, b.vy - uy);
	*/
	
	
	SetORCAConstraint(a, j, -Nx, -Ny, -Nx * (a.vx + ux) - Ny * (a.vy + uy));
	
	SetORCAConstraint(b, i, Nx, Ny, Nx * (b.vx - ux) + Ny * (b.vy - uy));
}

void ORCASolver::ComputeNewVelocities()
{

	for (int i = 0; i < num; i++)
	{		
		Agent& a = agents[i];
		
		for(int k = 0; k < a.nearbyCount; k++)
		{
			int j = a.nearbyAgents[k];
			
			if(i < j)
			{
				computeORCAConstraints(i, j);
			}
		}
		
		/*if (a.vx_pref * a.vx_pref + a.vy_pref * a.vy_pref < EPS)
		{
			a.vx_new = 0.f;
			a.vy_new = 0.f;
			continue;
		}*/
		
		solver.Reset();
		UE_LOG(LogRVOTest, VeryVerbose, TEXT("vxpref %f %f"), a.vx_pref, a.vy_pref);
		solver.SetDestination(a.vx_pref, a.vy_pref);
		for (int j = 0; j < a.nearbyCount; j++)
		{
			float A = a.ORCAA[j];
			float B = a.ORCAB[j];
			float C = a.ORCAC[j];
			
			//a.nearbyAgents[j] == i shouldn't happen but just to be safe
			if(fabs(A) < EPS && fabs(B) < EPS || a.nearbyAgents[j] == i)
			{
				continue;
			}
			
			solver.AddConstraintLinear(A, B, C);
			
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