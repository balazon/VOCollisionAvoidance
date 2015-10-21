// Fill out your copyright notice in the Description page of Project Settings.


//#include "RVOTest.h"


#include "ORCASolver.h"

#include "MathUtils.h"
#include <cmath>
#include <utility>

#include "CPLPSolver.h"

#include "SVGExporter.h"

#include <fstream>


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

bool ORCASolver::IsAgentNeighbour(int i, int j)
{
	Agent& a = agents[i];
	for (int k = 0; k < a.nearbyCount; k++)
	{
		if (a.nearbyAgents[k] == j)
		{
			
			return true;
		}
	}
	return false;
}



void ORCASolver::SetORCAConstraint(Agent& a, int j, float A, float B, float C)
{
	for (int i = 0; i < a.nearbyCount; i++)
	{
		if (a.nearbyAgents[i] == j)
		{
			a.ORCAA[i] = A;
			a.ORCAB[i] = B;
			a.ORCAC[i] = C;
			//UE_LOG(LogRVOTest, Warning, TEXT(" i %d"), j);

			return;
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

	//v^{opt}_A - v^{opt}_B which is vrel when v^{opt}_X = v_X for all X
	float vrelx = a.vx - b.vx;
	float vrely = a.vy - b.vy;


	float distSq = ABx * ABx + ABy* ABy;
	if (distSq > (a.maxVelocityMagnitude + b.maxVelocityMagnitude) * (a.maxVelocityMagnitude + b.maxVelocityMagnitude) * T * T)
	{
		SetORCAConstraint(a, j, 0.f, 0.f, 0.f);

		SetORCAConstraint(b, i, 0.f, 0.f, 0.f);

		return;
	}


	//v_opt is outside the VO
	bool outside = false;



	

	bool pqcircle = BMU::IntersectCircleCircle(ABx, ABy, R, ABx * .5f, ABy * .5f, .5f * sqrtf(ABx * ABx + ABy * ABy), Px, Py, Qx, Qy);
	//this is needed because actual agent radiuses are usually smaller than what is set (1.5, or 2x actual size)
	// so in dense situations close units are already "colliding" with the artificial radius -> no circle intersection
	while (!pqcircle || fabs(Px) < EPS && fabs(Py) < EPS || fabs(Qx) < EPS && fabs(Qy) < EPS)
	{
		R *= .9f;
		r = R / T;
		pqcircle = BMU::IntersectCircleCircle(ABx, ABy, R, ABx * .5f, ABy * .5f, .5f * sqrtf(ABx * ABx + ABy * ABy), Px, Py, Qx, Qy);
	}

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

	if (BMU::isnanf(Npx) || BMU::isnanf(Npy) || BMU::isnanf(Nqx) || BMU::isnanf(Nqy) || fabs(Npx) < EPS && fabs(Npy) < EPS || fabs(Nqx) < EPS && fabs(Nqy) < EPS)
	{
		UE_LOG(LogRVOTest, Warning, TEXT("BAM"));

	}


	//G, H points are not needed, but they are O's orthogonal projections to AP, and AQ (or the little circle's touching point)
	//they are the 'g' and 'h' in variable names below: Aog, Aoh, ..

	


	//outside QAP angle area

	if (Npx * vrelx + Npy * vrely > 0 || Nqx * vrelx + Nqy * vrely > 0)
	{
		//no chance for collision in this frame
		/*if (Npx * vrelx + Npy * vrely > a.maxAccMagnitude + b.maxAccMagnitude || Nqx * vrelx + Nqy * vrely > a.maxAccMagnitude + b.maxAccMagnitude )
		{
			SetORCAConstraint(a, j, 0.f, 0.f, 0.f);

			SetORCAConstraint(b, i, 0.f, 0.f, 0.f);

			return;
		}*/
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
		if (!orth)
		{
			orth = BMU::OrthogonalProjectionOfPointOnCircle(Ox, Oy, r, Ox + 2.f * Npx * EPS, Oy + 2.f * Npy * EPS, Sx, Sy);
			if (!orth)
			{
				UE_LOG(LogRVOTest, Warning, TEXT("POINTCIRCLE"));
				orth = BMU::OrthogonalProjectionOfPointOnCircle(Ox, Oy, r, Ox + 2.f * Npx * EPS, Oy + 2.f * Npy * EPS, Sx, Sy);
			}
		}
		Nx = Sx - Ox;
		Ny = Sy - Oy;
		outside = outside || ((vrelx - Ox) * (vrelx - Ox) + (vrely - Oy) * (vrely - Oy) > r * r);


		//no chance for collision in this frame
		/*if (outside && (vrelx - Sx) * (vrelx - Sx) + (vrely - Sy) * (vrely - Sy) > (a.maxAccMagnitude + b.maxAccMagnitude) * (a.maxAccMagnitude + b.maxAccMagnitude))
		{
			SetORCAConstraint(a, j, 0.f, 0.f, 0.f);

			SetORCAConstraint(b, i, 0.f, 0.f, 0.f);

			return;
		}*/

		float Nlrec = 1.f / sqrtf(Nx * Nx + Ny * Ny);
		Nx *= Nlrec;
		Ny *= Nlrec;
	}
	

	if (fabs(Nx) < EPS && fabs(Ny) < EPS || BMU::isnanf(Nx) || BMU::isnanf(Ny))
	{
		UE_LOG(LogRVOTest, Warning, TEXT("ORCALIN: %f %f"), Nx, Ny);
	}

	float ux = Sx - vrelx;
	float uy = Sy - vrely;

	if (!outside)
	{
		ux *= .5f;
		uy *= .5f;
	}
	/*ux += Nx * 1.f;
	uy += Ny * 1.f;*/

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

	
	//UE_LOG(LogRVOTest, Warning, TEXT("ij: %d %d \n"), i, j);

	SetORCAConstraint(a, j, A1, B1, C1);

	SetORCAConstraint(b, i, A2, B2, C2);
}

void ORCASolver::ComputeNewVelocities()
{
	for (int i = 0; i < num; i++)
	{
		Agent& a = agents[i];

		for (int k = 0; k < a.nearbyCount; k++)
		{
			int j = a.nearbyAgents[k];

			//TODO what changes when i < j is replaced with true?
			if (i < j || !IsAgentNeighbour(j, i))
			{
				
				computeORCAConstraints(i, j);
				//UE_LOG(LogRVOTest, Warning, TEXT("compute after ij: %d %d \n"), i, j);

			}
		}
		

		solver.Reset();
		UE_LOG(LogRVOTest, VeryVerbose, TEXT("vxpref %f %f"), a.vx_pref, a.vy_pref);
		solver.SetDestination(a.vx_pref, a.vy_pref);
		for (int j = 0; j < a.nearbyCount; j++)
		{
			float A = a.ORCAA[j];
			float B = a.ORCAB[j];
			float C = a.ORCAC[j];
			
			//a.nearbyAgents[j] == i shouldn't happen but just to be safe
			if (fabs(A) < EPS && fabs(B) < EPS || a.nearbyAgents[j] == i )
			{
				continue;
			}

			if (BMU::isnanf(A) || BMU::isnanf(B) || BMU::isnanf(C))
			{
				UE_LOG(LogRVOTest, Warning, TEXT("NAN ij: %d %d, ABC: %f %f %f"), i, a.nearbyAgents[j], A, B, C);

				UE_LOG(LogRVOTest, Warning, TEXT("NAN i: %d near: "), i);

				for (int k = 0; k < a.nearbyCount; k++)
				{
					UE_LOG(LogRVOTest, Warning, TEXT(" %d"), a.nearbyAgents[k]);

				}
				
				UE_LOG(LogRVOTest, Warning, TEXT("NAN j: %d near: "), a.nearbyAgents[j]);

				for (int k = 0; k < a.nearbyCount; k++)
				{
					UE_LOG(LogRVOTest, Warning, TEXT(" %d"), agents[a.nearbyAgents[j]].nearbyAgents[k]);

				}
				
			}

			UE_LOG(LogRVOTest, VeryVerbose, TEXT("vxpref %f %f"), a.vx_pref, a.vy_pref);

			solver.AddConstraintLinear(A, B, C);

		}


		solver.AddConstraintCircle(a.vx, a.vy, a.maxAccMagnitude, true);
		solver.AddConstraintCircle(0.f, 0.f, a.maxVelocityMagnitude, true);



		solver.Solve(a.vx_new, a.vy_new);

		if (solver.usedSafest)
		{
			float d = solver.usedDInSafest;
			for (int j = 0; j < 11; j++)
			{
				float printd = d * (float)j / 10.f;
				
				char buffer[5];
				sprintf(buffer, "%d", j);
				SVGExporter::writeUnitORCAs("D:/Bala/Unreal Projects/RVOTest/test" + std::string{buffer} + ".svg", this, num, i, printd);
			}

			UE_LOG(LogRVOTest, Warning, TEXT("SVG's done"));
			BMU::debug = true;
			solver.debug = true;
			solver.Solve(a.vx_new, a.vy_new);
			BMU::debug = false;
			solver.debug = false;
		}


		if (a.vx_new * a.vx_new + a.vy_new * a.vy_new > a.maxVelocityMagnitude * (a.maxVelocityMagnitude + 2.f * EPS))
		{
			
			a.vx_new *= 0.999f;
			a.vy_new *= 0.999f;

			if (a.vx_new * a.vx_new + a.vy_new * a.vy_new > a.maxVelocityMagnitude * (a.maxVelocityMagnitude + 2 * EPS))
			{

				//SVGExporter::writeUnitORCAs("D:/Bala/Unreal Projects/RVOTest/test.svg", this, num, i);
				UE_LOG(LogRVOTest, Warning, TEXT("a.v: %f %f, a.vnew : %f %f, a.vmax: %f"), a.vx, a.vy, a.vx_new, a.vy_new, a.maxVelocityMagnitude);
				BMU::debug = true;
				solver.debug = true;
				solver.Solve(a.vx_new, a.vy_new);
				BMU::debug = false;
				solver.debug = false;
			}

		}

		float axnew = a.vx_new - a.vx;
		float aynew = a.vy_new - a.vy;
		if (axnew * axnew + aynew * aynew > a.maxAccMagnitude * (a.maxAccMagnitude + 2.f * EPS))
		{

			axnew *= 0.99f;
			aynew *= 0.99f;
			a.vx_new = a.vx + axnew;
			a.vy_new = a.vy + aynew;

			if ((a.vx_new - a.vx) * (a.vx_new - a.vx) + (a.vy_new - a.vy) * (a.vy_new - a.vy) > a.maxAccMagnitude * (a.maxAccMagnitude + 2 * EPS))
			{
				//SVGExporter::writeUnitORCAs("D:/Bala/Unreal Projects/RVOTest/test.svg", this, num, i);
				UE_LOG(LogRVOTest, Warning, TEXT("a.v %f %f, a.vnew %f %f, a.maxacc : %f"), a.vx, a.vy, a.vx_new, a.vy_new, a.maxAccMagnitude);
				BMU::debug = true;
				solver.debug = true;
				solver.Solve(a.vx_new, a.vy_new);
				BMU::debug = false;
				solver.debug = false;
			}

		}

		if (BMU::isnanf(a.vx_new) || BMU::isnanf(a.vy_new))
		{
			//FString dir{ "D:/Bala/Unreal Projects/RVOTest/test.svg" };
			
			SVGExporter::writeUnitORCAs("D:/Bala/Unreal Projects/RVOTest/test.svg", this, num, i);
			UE_LOG(LogRVOTest, Warning, TEXT("BAM\n"));
			UE_LOG(LogRVOTest, Warning, TEXT("a.v: %f %f, a.vnew : %f %f, a.vmax: %f"), a.vx, a.vy, a.vx_new, a.vy_new, a.maxVelocityMagnitude);
			UE_LOG(LogRVOTest, Warning, TEXT("a.v %f %f, a.vnew %f %f, a.maxacc : %f"), a.vx, a.vy, a.vx_new, a.vy_new, a.maxAccMagnitude);

			BMU::debug = true;
			solver.debug = true;
			solver.Solve(a.vx_new, a.vy_new);
			BMU::debug = false;
			solver.debug = false;
			if (BMU::isnanf(a.vx_new) || BMU::isnanf(a.vy_new))
			{
				UE_LOG(LogRVOTest, VeryVerbose, TEXT("BAM confirm\n"));
			}
		}

		if (BMU::isnanf(a.vx_new) || BMU::isnanf(a.vy_new))
		{
			a.vx_new = a.vx;
			a.vy_new = a.vy;
		}
	}
}

void ORCASolver::SetDebugging(bool on)
{
	solver.debug = on;
	BMU::debug = on;
}