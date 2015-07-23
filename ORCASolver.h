#pragma once

#include <vector>
#include <map>

#define CA_MAXAGENTS (400)

#define CA_MAXNEARBY (5)

struct Agent
{
	float x, y;
	float vx, vy;
	float r;
	float vx_pref;
	float vy_pref;
	float vx_new;
	float vy_new;
	int nearbyAgents[CA_MAXNEARBY];
	float ux[CA_MAXNEARBY];
	float uy[CA_MAXNEARBY];
	
	//constraints? time? (acceleration - paralelogram?) !!
	
	Agent();
	Agent(float x, float y, float vx, float vy, float r, float vx_pref, float vy_pref);
	
	~Agent();
};



class ORCASolver
{
public:
	ORCASolver();
	~ORCASolver();
	
	void ClearNeighbours(int i);
	void SetAgentsNearby(int i, int j);
	bool AreAgentsNeighbours(int i, int j);
	void SetUVector(int i, int j, float ux, float uy);
	
	//void setAgentState(float x, float y, float vx, float vy, float r, float vx_pref, float vy_pref)
	
	//reference, because when an agent is removed, our id could change so that there are no gaps in array
	Agent& GetAgent(int& id);
	
	//returns id of agent
	int AddAgent();
	void RemoveAgent(int id);
	void ClearAgents();
	
	void SetParameters(float T);
	
	//new velocities which hopefully help avoid collisions
	void ComputeNewVelocities();
	
private:

	//used for calculating the limited VO (t < T)
	float T;
	
	int num;
	Agent agents[CA_MAXAGENTS];
	
	//void computeORCAConstraint(float Ax, float Ay, float Bx, float By, float Vax, float Vay, float Vbx, float Vby, float Ra, float Rb)
	
	//compute u vector: u for A agent with respect to B, and -u for agent B with respect to A
	//void computeSmallestChangeRequired
	
	std::map<int, int> replacingIds;
	
	void computeSmallestChangeVectors(int i, int j);
};

