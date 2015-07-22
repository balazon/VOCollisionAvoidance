#include "ORCASolver.h"

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


void ORCASolver::ComputeNewVelocities()
{
	replacingIds.clear();
	
	//TODO not implemented yet - 
}